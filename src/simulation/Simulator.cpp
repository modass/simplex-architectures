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

    bool Simulator::simulateSafety(const Point& ctrlInput) {
        roots.clear();

        for ( const auto& [LocPtr, samples] : mLastStates ) {
            for ( auto sample : samples ) {
                setCtrlValue(sample, ctrlInput);
                // create intervals representing the initial state
                std::vector<carl::Interval<Number>> intervals;
                for ( Eigen::Index i = 0; i < sample.dimension(); ++i ) {
                    intervals.emplace_back( carl::Interval<Number>( sample.at( i ) ) );
                }
                auto initialBox = hypro::Condition<Number>{ intervals };
                typename hypro::HybridAutomaton<Number>::locationConditionMap initialStates;
                initialStates[LocPtr] = initialBox;
                mAutomaton.setInitialStates( initialStates );
                auto sampleRoots = hypro::makeRoots<Representation>( mAutomaton );
                // add roots for this sample to global reachtree
                for ( auto&& sr : sampleRoots ) {
                    roots.emplace_back( std::move( sr ) );
                }
                std::cout << "[Simulator] Add sample " << sample << " for simulation." << std::endl;
            }
        }

        // call simulation as reachability analysis for a maximal time duration of 1 (cycle time)
        // copy settings to adjust global time etc.
        mSettings.rFixedParameters().globalTimeHorizon = mCycleTime;
        mSettings.rFixedParameters().localTimeHorizon = carl::convert<double, hypro::tNumber>( mCycleTime );
        mSettings.rFixedParameters().jumpDepth = 2 * std::ceil( mCycleTime / carl::convert<hypro::tNumber, double>( mSettings.strategy().front().timeStep ) );
        // analysis
        auto reacher = hypro::reachability::Reach<Representation>( mAutomaton, mSettings.fixedParameters(),
                                                                   mSettings.strategy().front(), roots );
        auto result = reacher.computeForwardReachability();

        std::cout << "[Simulator] simulate safety result: " << result << std::endl;
        return (result == hypro::REACHABILITY_RESULT::SAFE);
    }

    void Simulator::update(const Point& ctrlInput, const Point& nextObservation) {
        simulateSafety(ctrlInput);
        for (auto &root: roots) {
            cutoffControllerJumps(&root);
        }

        std::map<LocPtr, Box> samplesBoxes;
        Matrix constraints = Matrix::Zero( 2, 5 );
        Vector constants = Vector::Zero( 2 );
        // assign constraints: x1, x2 = observation, tick = cycle time
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
                    if ( containment != hypro::CONTAINMENT::NO ) {
                        std::cout << "[Simulator] New sample: " << result << std::endl;
                        if ( samplesBoxes.find( n.getLocation() ) != samplesBoxes.end() ) {
                            samplesBoxes[n.getLocation()] = samplesBoxes[n.getLocation()].unite( result );
                        } else {
                            samplesBoxes[n.getLocation()] = result;
                        }
                    }
                }
            }
        }

        std::cout << "[Simulator] Observation: " << nextObservation << std::endl;
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
                std::cout << "[Simulator] Add samples " << mLastStates[LocPtr] << " to mLastStates (Location: " << LocPtr->getName() << ")" << std::endl;
                assert( mLastStates[LocPtr].size() <= 2 );
            }
        }

    }

    void Simulator::setCtrlValue(Point &sample, const Point &ctrlInput) {
        // augment state with controller input
        sample.at( 2 ) = ctrlInput.at( 0 );
    }
}