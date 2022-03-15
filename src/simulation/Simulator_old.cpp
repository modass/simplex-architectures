/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 16.02.22.
 */

#include "Simulator_old.h"

namespace simplexArchitectures {

void Simulator::pointify() {
  std::cout << "[Simulator] Pointify" << std::endl;
  std::map<LocPtr, Box> samplesBoxes;
  // create constraints which fix the observation
  Matrix constraints = Matrix::Zero( 2, 5 );
  Vector constants = Vector::Zero( 2 );
  // assign constraints: x1, x2 = observation, tick = cycle time
  /*
  // x1
  constraints( 0, 0 ) = 1;
  constraints( 1, 0 ) = -1;
  constants( 0 ) = observation.at( 0 );
  constants( 1 ) = -observation.at( 0 );
  // x2
  constraints( 2, 1 ) = 1;
  constraints( 3, 1 ) = -1;
  constants( 2 ) = observation.at( 1 );
  constants( 3 ) = -observation.at( 1 );
   */
  // tick
  constraints( 0, 4 ) = 1;
  constraints( 1, 4 ) = -1;
  //		constants( 0 ) = mCycleTime;
  //		constants( 1 ) = -mCycleTime;
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
  // create an artificial observation
  std::mt19937 generator;
  std::uniform_int_distribution<std::size_t> LocPtr_dist{ 0, samplesBoxes.size() - 1 };
  std::size_t chosenLocPtr = LocPtr_dist( generator );
  LocPtr LocPtrptr = std::next( samplesBoxes.begin(), chosenLocPtr )->first;
  Point observation = samplesBoxes.at( LocPtrptr ).vertices().front().projectOn( { 0, 1 } );
  std::cout << "[Simulator] Observation: " << observation << std::endl;
  // build constraints which represent the observation
  constraints = Matrix::Zero( 4, 5 );
  constants = Vector::Zero( 4 );
  // assign constraints: x1, x2 = observation
  // x1
  constraints( 0, 0 ) = 1;
  constraints( 1, 0 ) = -1;
  constants( 0 ) = observation.at( 0 );
  constants( 1 ) = -observation.at( 0 );
  // x2
  constraints( 2, 1 ) = 1;
  constraints( 3, 1 ) = -1;
  constants( 2 ) = observation.at( 1 );
  constants( 3 ) = -observation.at( 1 );
  // filter sample boxes for observation
  for ( auto& [_, box] : samplesBoxes ) {
    box = box.intersectHalfspaces( constraints, constants );
  }

  // collect concrete samples from samplesboxes
  mLastStates.clear();
  for ( auto& [LocPtr, box] : samplesBoxes ) {
    auto tmp = box.vertices();
    if ( !tmp.empty() ) {
      mLastStates[LocPtr] = std::set<Point>( tmp.begin(), tmp.end() );
      std::cout << "[Simulator] Add samples " << mLastStates[LocPtr] << " to mLastStates (LocPtration: " << LocPtr->getName() << ")" << std::endl;
      assert( mLastStates[LocPtr].size() <= 2 );
    }
  }
}

bool Simulator::simulate( bool updateBaseController ) {
  // augment last state with controller update (e.g. set u), set cLocPtrk to zero
  // set mLastState as initial state of the model
  // run simulation for cycle time (here: 1), pointify to potential new states
  // return computed simulation trace, set mPotentialNewState & LocPtration
  // TODO what if the plant is non-deterministic?

  ControllerUpdate ctrlInput;
  if ( updateBaseController ) {
    // ctrlInput = mBaseController.generateInput();
    //  TODO add assertion which checks that all controller LocPtrations are the same in the samples
    auto LocPtrName = mLastStates.begin()->first->getName();
    ctrlInput.loc = mLastStates.begin()->first;
    if ( LocPtrName.find( "_on_" ) != std::string::npos ) {
      ctrlInput.val = Point( Vector::Ones( 1 ) * 0.0002 );
    } else {
      ctrlInput.val = Point( Vector::Zero( 1 ) );
    }
    std::cout << "[Simulator] Generated output for base controller, u = " << ctrlInput.val << std::endl;
  } else {
    ctrlInput = mAdvancedController.generateInput();
    std::cout << "[Simulator] Generated output for advanced controller, u = " << ctrlInput.val << std::endl;
  }

  // cleanup roots for new simulation run
  roots.clear();

  for ( const auto& [LocPtr, samples] : mLastStates ) {
    for ( auto sample : samples ) {
      // augment state with controller input
      sample.at( 2 ) = ctrlInput.val.at( 0 );
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
  // copy settings to adjust jump depth etc.
  mSettings.rFixedParameters().localTimeHorizon = carl::convert<double, hypro::tNumber>( mCycleTime );
  mSettings.rFixedParameters().jumpDepth = 2 * std::ceil( mCycleTime / carl::convert<hypro::tNumber, double>( mSettings.strategy().front().timeStep ) );
  // analysis
  auto reacher = hypro::reachability::Reach<Representation>( mAutomaton, mSettings.fixedParameters(),
                                                            mSettings.strategy().front(), roots );
  auto result = reacher.computeForwardReachability();

  // cutoff after cycle time
  // TODO add functionality to run reachability analysis for a bounded global time
  // Workaround: compute larger set, post-process: cutoff all nodes in the tree reachable via a controller-jump.
  // note: in the system there is no trajectory with length longer than cycle time since the controller is definitely invoked after this time on any execution path.
  for (auto &root: roots) {
    cutoffControllerJumps(&root);
  }

  // return safety result
  // dbg
  std::cout << "[Simulator] simulate safety result: " << result << std::endl;
  return (result == hypro::REACHABILITY_RESULT::SAFE);
}

}

