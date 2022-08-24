//
// Created by bmaderbacher on 08.03.22.
//

#include "Simulator.h"

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

        for ( const auto& [l, samples] : mLastStates ) {
            for ( auto sample : samples ) {
                setCtrlValue(sample, ctrlInput);
                LocPtr newLocation = l;
                if(mLocationUpdate) {
                  newLocation = mLocationUpdate(ctrlInput,l);
                }
                // create intervals representing the initial state
                std::vector<carl::Interval<Number>> intervals;
                for ( Eigen::Index i = 0; i < sample.dimension(); ++i ) {
                    intervals.emplace_back( carl::Interval<Number>( sample.at( i ) ) );
                }
                auto initialBox = hypro::Condition<Number>{ intervals };
                typename hypro::HybridAutomaton<Number>::locationConditionMap initialStates;
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
      spdlog::debug("Update simulator");
      isSafe( ctrlInput );
        for (auto &root: roots) {
            cutoffControllerJumps(&root);
        }

        std::map<LocPtr, Box> samplesBoxes;
        Matrix constraints = Matrix::Zero( 2, 5 );
        Vector constants = Vector::Zero( 2 );
        // assign constraints:  tick = 0
        // tick
        constraints( 0, 4 ) = 1;
        constraints( 1, 4 ) = -1;
        constants( 0 ) = 0;
        constants( 1 ) = -0;
        // collect all leaf nodes that agree with the cycle time
        for ( auto& r : roots ) {
            for ( auto& n : hypro::preorder( r ) ) {
                if ( n.isLeaf() ) {
                    // I don't think we really need this check. We only consider initial sets of nodes that where reached by resetting the cLocPtrk to zero.
                    auto [containment, result] = n.getInitialSet().satisfiesHalfspaces( constraints, constants );
                    if(containment != hypro::CONTAINMENT::FULL) {
                      std::stringstream ss,sss;
                      ss << n.getInitialSet();
                      sss << n.getParent()->getInitialSet();
                      spdlog::warn("Leaf node ({}, parent {}, parent initial: {}) initial set {} should be fully contained in tick = 0, but is actually not.", n.getLocation()->getName(), n.getParent()->getLocation()->getName(), sss.str(), ss.str());
                      ss.str(std::string());
                      ss << n.getPath();
                      spdlog::warn("Node path: {}", ss.str());
                      ss.str(std::string());
                      for(const auto& s : n.getFlowpipe()) {
                        ss << s << "\n";
                      }
                      // std::cout << "Node flowpipe:\n" << ss.str() << std::endl;
                      spdlog::warn("Node flags: timelock: {}, bad state: {}, has fixed point: {}, is on Zeno-cycle: {}", n.hasTimelock(), n.intersectedUnsafeRegion(), n.hasFixedPoint()==hypro::TRIBOOL::TRUE, n.isOnZenoCycle());
                      throw std::logic_error("Leaf node initial set " + ss.str() + " should be fully contained in tick = 0, but is actually not.");
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

        // build constraints which represent the observation
        constraints = Matrix::Zero( 4, 5 );
        constants = Vector::Zero( 4 );
        // assign constraints: x1, x2 = observation
        // x1
        constraints( 0, 0 ) = 1;
        constraints( 1, 0 ) = -1;
        constants( 0 ) = nextObservation.at( 0 );
        constants( 1 ) = -nextObservation.at( 0 );
        // x2
        constraints( 2, 1 ) = 1;
        constraints( 3, 1 ) = -1;
        constants( 2 ) = nextObservation.at( 1 );
        constants( 3 ) = -nextObservation.at( 1 );
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
                // std::cout << "[Simulator] Add samples " << mLastStates[LocPtr] << " to mLastStates (Location: " <<
                // LocPtr->getName() << ")" << std::endl;
                assert( mLastStates[LocPtr].size() <= 2 );
            }
        }
        spdlog::debug("Update simulator done.");
    }

    void Simulator::setCtrlValue(Point &sample, const Point &ctrlInput) {
        // augment state with controller input
        // sample.at( 2 ) = ctrlInput.at( 0 );
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