//
// Created by bmaderbacher on 08.03.22.
//

#include "Simulator.h"
#include <spdlog/fmt/bundled/ostream.h>

namespace simplexArchitectures {

    Point Simulator::getBaseControllerOutput() {
        //  TODO add assertion which checks that all controller Locations are the same in the samples
        auto LocPtrName = mLastStates.begin()->first->getName();
        if ( LocPtrName.find( "_on_" ) != std::string::npos ) {
            return Point( Vector::Ones( 1 ) * 0.0002 );
        } else {
            return Point( Vector::Zero( 1 ) );
        }
    }

    hypro::TRIBOOL Simulator::isSafe(const Point& ctrlInput) {
        roots.clear();

//        spdlog::trace("Simulate for samples in {} locations",mLastStates.size());
        for ( const auto& [l, samples] : mLastStates ) {
            for ( auto sample : samples ) {
            //                spdlog::trace("Simulate from {} starting in location {}", sample, l->getName());
            setCtrlValue( sample, ctrlInput );
            LocPtr newLocation = l;
            // spdlog::trace("Location before update: {}", l->getName());
            if ( mLocationUpdate ) {
              newLocation = mLocationUpdate( ctrlInput, l );
            }
            // spdlog::trace("Updated sample: {} in location {}",sample,newLocation->getName());
            //  create intervals representing the initial state
            std::vector<carl::Interval<Number>> intervals;
            for ( Eigen::Index i = 0; i < sample.dimension(); ++i ) {
              intervals.emplace_back( carl::Interval<Number>( sample.at( i ) ) );
            }
            auto                                     initialBox = hypro::Condition<Number>{ intervals };
            typename Automaton::locationConditionMap initialStates;
            initialStates[newLocation] = initialBox;
            mAutomaton.setInitialStates( initialStates );
            auto sampleRoots = hypro::makeRoots<Representation>( mAutomaton );
            // add roots for this sample to global reachtree
            for ( auto&& sr : sampleRoots ) {
              roots.emplace_back( std::move( sr ) );
            }
            // std::cout << "[Simulator] Add sample " << sample << " for simulation." << std::endl;
          }
        }

        // call simulation as reachability analysis for a maximal time duration of 1 (cycle time)
        // copy settings to adjust global time etc.
        mSettings.rFixedParameters().globalTimeHorizon = mCycleTime * 1.5;
        mSettings.rFixedParameters().localTimeHorizon  = carl::convert<double, hypro::tNumber>( mCycleTime );
        mSettings.rFixedParameters().jumpDepth =
            2 *
            std::ceil( mCycleTime / carl::convert<hypro::tNumber, double>( mSettings.strategy().front().timeStep ) );
        mSettings.rStrategy().front().detectJumpFixedPoints              = false;
        mSettings.rStrategy().front().detectContinuousFixedPointsLocally = false;
        mSettings.rStrategy().front().detectFixedPointsByCoverage        = false;
        mSettings.rStrategy().front().numberSetsForContinuousCoverage    = 0;
        mSettings.rStrategy().front().detectZenoBehavior                 = true;
        // analysis
        auto reacher =
            ReachabilityAnalyzer( mAutomaton, mSettings.fixedParameters(), mSettings.strategy().front(), roots );
        auto result = reacher.computeForwardReachability();

        // std::cout << "[Simulator] simulate safety result: " << result << std::endl;
        if ( result != hypro::REACHABILITY_RESULT::SAFE ) {
          return hypro::TRIBOOL::FALSE;
        }

        // collect unknown samples, i.e., new samples which have not yet been stored in the storage
        // For those samples we do not know whether they are safe for the base controller.
        unknownSamples.clear();
        auto isSafe     = hypro::TRIBOOL::TRUE;
        auto nextStates = potentialNextStates();
        //spdlog::trace("Have {} potential next states after cutoff.", nextStates.size());
        for(const auto& [loc,setVector] : nextStates) {
          for(const auto& set : setVector) {
            if(!mStorage.isContained(loc->getName(),set)) {
              isSafe = hypro::TRIBOOL::NSET;
              if(unknownSamples.find(loc) == std::end(unknownSamples)) {
                unknownSamples[loc] = std::vector<Representation>{};
              }
              unknownSamples[loc].emplace_back(set);
            }
          }
        }
        return isSafe;
    }

    void Simulator::update(const Point& ctrlInput, const Point& nextObservation) {
      spdlog::debug(
          "Update simulator with ctrl input {}, old states {}, old location {}, and the current observation {}",
          ctrlInput, mLastStates, mLastStates.begin()->first->getName(), nextObservation );
      auto safe = isSafe( ctrlInput );

      if ( safe == hypro::TRIBOOL::FALSE ) {
        spdlog::warn( "Simulator updated with unsafe input ({}). Print details of the reachability analysis:", safe );
        // This might happen after the cutoff and thus be spurious.
        for ( auto& r : roots ) {
          for ( auto& n : hypro::preorder( r ) ) {
            spdlog::warn( "Process node with location {}, leaf: {}, initial set {}, flow pipe {}",
                          n.getLocation()->getName(), n.isLeaf(), n.getInitialSet(), n.getFlowpipe() );
          }
        }
        mStorage.plot( "error_storage" );
        throw std::logic_error( "An update to unsafe states should not happen." );
      }
      if (safe == hypro::TRIBOOL::NSET) {
        spdlog::warn("Simulator updated with input that might visit unexplored states!");
        // This can only happen, if isSafe() ends in states that are not in the storage yet.
        spdlog::warn("locations with unknown samples:");
        for(auto u : unknownSamples) {
          spdlog::warn("{}", u.first->getName());
        }
        spdlog::warn("The unknown samples are: {}", unknownSamples);
//        throw std::logic_error("Cannot detect new states during update.");
        // This seems to some times happen, most likely due to numerical impression.
        // Show the warning, but continue with the execution.
      }
        for (auto &root: roots) {
            cutoffControllerJumps(&root);
        }


        std::map<LocPtr, Box> samplesBoxes;
        Matrix constraints = Matrix::Zero( 2, mAutomaton.dimension() );
        Vector constants = Vector::Zero( 2 );
        // assign constraints:  tick = 0
        // tick
        constraints( 0, mCycleTimeDimension ) = 1;
        constraints( 1, mCycleTimeDimension ) = -1;
        constants( 0 ) = 0;
        constants( 1 ) = -0;
        // collect all leaf nodes that agree with the cycle time
        for ( auto& r : roots ) {
            for ( auto& n : hypro::preorder( r ) ) {
            if ( n.isLeaf() ) {
              // I don't think we really need this check. We only consider initial sets of nodes
              // that where reached by resetting the clock to zero.
              auto [containment, result] = n.getInitialSet().satisfiesHalfspaces( constraints, constants );
              if ( containment != hypro::CONTAINMENT::FULL ) {
                std::stringstream ss, sss;
                ss << n.getInitialSet();
                sss << n.getParent()->getInitialSet();
                spdlog::warn(
                    "Leaf node ({}, parent {}, parent initial: {}) initial set {} should be fully contained in tick = "
                    "0, but is actually not.",
                    n.getLocation()->getName(), n.getParent()->getLocation()->getName(), sss.str(), ss.str() );
                ss.str( std::string() );
                ss << n.getPath();
                spdlog::warn( "Node path: {}", ss.str() );
                ss.str( std::string() );
                for ( const auto& s : n.getFlowpipe() ) {
                  ss << s << "\n";
                }
                spdlog::warn( "Root node initial set: {}", r.getInitialSet() );
                // std::cout << "Node flowpipe:\n" << ss.str() << std::endl;
                spdlog::warn( "Node flags: timelock: {}, bad state: {}, has fixed point: {}, is on Zeno-cycle: {}",
                              n.hasTimelock(), n.intersectedUnsafeRegion(), n.hasFixedPoint() == hypro::TRIBOOL::TRUE,
                              n.isOnZenoCycle() );
                // throw std::logic_error("Leaf node initial set " + ss.str() + " should be fully contained in tick = 0,
                // but is actually not.");
                continue;
              }
              if ( containment != hypro::CONTAINMENT::NO ) {
                // std::cout << "[Simulator] New sample: " << result << std::endl;
                if ( samplesBoxes.find( n.getLocation() ) != samplesBoxes.end() ) {
                  samplesBoxes[n.getLocation()] = samplesBoxes[n.getLocation()].unite( result );
                } else {
                  samplesBoxes[n.getLocation()] = result;
                }
              }
            }
          }
        }

        // build constraints which represent the observation, only consider dimensions relevant
        std::size_t dim = mObservationDimensions.size();
        constraints = Matrix::Zero( 2*dim, mAutomaton.dimension() );
        constants = Vector::Zero( 2*dim );
        for(std::size_t i = 0; i < dim; ++i) {
          constraints( 2*i, mObservationDimensions[i] ) = 1;
          constraints( (2*i)+1, mObservationDimensions[i] ) = -1;
          constants( 2*i ) = nextObservation[i];
          constants( (2*i)+1 ) = -nextObservation[i];
        }
        // filter sample boxes for observation
        for ( auto& [_, box] : samplesBoxes ) {
          box = box.intersectHalfspaces( constraints, constants );
        }

        // collect concrete samples from samplesBoxes
        mLastStates.clear();
        for ( auto& [LocPtr, box] : samplesBoxes ) {
            auto tmp = box.vertices();
            if ( !tmp.empty() ) {
                mLastStates[LocPtr] = std::set<Point>( tmp.begin(), tmp.end() );
                assert( mLastStates[LocPtr].size() <= 2 );
            }
        }
        if(mLastStates.empty()) {
          spdlog::warn("Reachability computation roots: {}", roots);
          throw std::logic_error("None of the simulated traces agrees with the actual real-world observation passed to the simulator.");
        }
        spdlog::debug("Update simulator done.");
    }

    void Simulator::setCtrlValue(Point &sample, const Point &ctrlInput) {
        // augment state with controller input
        for(Eigen::Index i = 0; i < mControlDimensions.size(); ++i) {
          sample[mControlDimensions[i]] = ctrlInput[i];
        }
    }

    std::map<LocPtr, std::vector<Box>> Simulator::potentialNextStates() {
        for (auto &root: roots) {
            cutoffControllerJumps(&root);
        }

       std::map<LocPtr, std::vector<Box>> samples;
        for ( const auto &r : roots ) {
            for ( const auto &n : hypro::preorder( r ) ) {
                if ( n.isLeaf() ) {
                    samples[n.getLocation()].push_back( n.getInitialSet() );
                }
            }
        }
        return samples;
    }
}