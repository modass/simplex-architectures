//
// Created by bmaderbacher on 05.07.23.
//

#include "ActionSequenceSimulator.h"
hypro::TRIBOOL simplexArchitectures::ActionSequenceSimulator::simulate(
    const std::vector<hypro::Label>& controllerActions,
    LocPtr initialLocation,
    const hypro::Condition<Number>& initialBox  ) {

  // Construct an automaton of the trace
  hypro::HybridAutomaton<Number> traceAutomaton;

  //pseudo location to ensure that traceAutomaton contains all labels for synchronization
  auto dummyLocation = traceAutomaton.createLocation();
  auto allLabels = mAutomaton.getLabels();
  for (const auto& l :  allLabels) {
    auto t = dummyLocation->createTransition(dummyLocation);
    t->addLabel(l);
  }

  auto loc0 = traceAutomaton.createLocation();
  loc0->setName( "trace_" + std::to_string( 0 ) );
  auto previousLocation = loc0;
  size_t it = 1;
  for (const auto& action: controllerActions) {
    auto loc = traceAutomaton.createLocation();
    loc->setName( "trace_" + std::to_string( it ) );
    auto t = previousLocation->createTransition(loc);
    t->addLabel(action);
    previousLocation = loc;
    it++;
  }



  // build composition
  mSimulationAutomaton.addAutomaton(std::move(traceAutomaton));

  auto initialLocationName = initialLocation->getName() + "_trace_0";
  auto newLocation = mSimulationAutomaton.getLocation(initialLocationName);
  typename Automaton::locationConditionMap initialStates;
  initialStates[newLocation] = initialBox;
  mSimulationAutomaton.setInitialStates( initialStates );

  mRoots = hypro::makeRoots<Representation>( mSimulationAutomaton );
  auto reacher =
      ReachabilityAnalyzer( mSimulationAutomaton, mSettings.fixedParameters(), mSettings.strategy().front(), mRoots );

  // set up callbacks which are used by hypro to access the octree
  std::size_t                                                   shortcuts = 0;
  std::function<bool( const Representation&, const Location* )> callback  = [this, &shortcuts]( const auto& set,
                                                                                               const auto  locptr ) {
    if ( mStorage.isContained( locptr->getName(), set ) ) {
      ++shortcuts;
      return true;
    }
    return false;
  };
  hypro::ReachabilityCallbacks<Representation, Location> callbackStructure{ callback };
  reacher.setCallbacks( callbackStructure );
  auto result = reacher.computeForwardReachability();
  if ( shortcuts > 0 ) {
    spdlog::trace( "Found {} fixed points by exploiting existing results.", shortcuts );
  }

  std::vector<std::vector<std::pair<LocPtr, Box>>> timeStepNodes(controllerActions.size());
  mTimeStepNodes.clear();
  mTimeStepNodes.resize(controllerActions.size());


  Matrix constraints = Matrix::Zero( 2, mAutomaton.dimension() );
  Vector constants = Vector::Zero( 2 );
  // assign constraints:  tick = mCycleTime
  // tick
  constraints( 0, mCycleTimeDimension ) = 1;
  constraints( 1, mCycleTimeDimension ) = -1;
  constants( 0 ) = mCycleTime;
  constants( 1 ) = -mCycleTime;

  // safe all states reachable after each time step
  for ( auto& r : mRoots ) {
    for ( auto& node : hypro::preorder( r ) ) {
      // This should capture all nodes that are reached by a sync transition or are root nodes
      if(node.getTransition() == nullptr || !node.getTransition()->getLabels().empty()){
        auto locationName = node.getLocation()->getName();
        for ( size_t k = 0; k < controllerActions.size(); ++k ) {
          auto traceK = "trace_"+std::to_string(k);
          if(locationName.find(traceK) != std::string::npos){
            mTimeStepNodes[k].emplace_back(node.getLocation(), node.getInitialSet());

            if (k>0) {
              auto flowpipe = node.getParent()->getFlowpipe();
              Box  lastBox  = flowpipe.at( flowpipe.size() - 1 );
              Box  box = lastBox.intersectHalfspaces( constraints, constants );
              auto t   = std::make_tuple( node.getParent()->getLocation(), box, controllerActions[k-1] );
              mPotentialActions[k-1].emplace_back( t );
            }
            break;
          }
        }
      }
    }
  }

  if (result == hypro::REACHABILITY_RESULT::SAFE) {
    return hypro::TRIBOOL::TRUE;
  } else {
    return hypro::TRIBOOL::FALSE;
  }
}
int simplexArchitectures::ActionSequenceSimulator::storageReachedAtTime() {
  int t = 0;
  for (auto &timeStep : mTimeStepNodes) {
    bool allReached = true;
    for (const auto& p: timeStep) {
      auto contained = mStorage.isContained(p.first->getName(), p.second);
      if (!contained) {
        allReached = false;
      }
    }
    if(allReached) {
      return t;
    }
    t++;
  }
  return -1; //There is no time step where all states are contained in the storage.
}
bool simplexArchitectures::ActionSequenceSimulator::wasStorageReached() {
  return storageReachedAtTime() >= 0;
}
std::vector<std::pair<std::pair<LocPtr, Box>, hypro::Label>>
simplexArchitectures::ActionSequenceSimulator::getStateActionPairs() {
  std::vector<std::pair<std::pair<LocPtr, Box>, hypro::Label>> result;

  auto sequenceLength = storageReachedAtTime();
  assert(sequenceLength>=0);

  for ( int t = 0; t < sequenceLength; ++t ) {
    auto stateActionPairs = mPotentialActions[t];
    for (const auto& p: stateActionPairs) {
      LocPtr originalLocation = mAutomaton.getLocation(extractOriginalLocationName(std::get<0>(p)->getName() ));
      Box box = std::get<1>(p);
      auto action = std::get<2>(p);
      result.emplace_back(std::make_pair( originalLocation, box), action);
    }
  }

  return result;
}
std::string simplexArchitectures::ActionSequenceSimulator::extractOriginalLocationName(
    const std::string& composedLocationName ) {
  auto pos = composedLocationName.find("_trace_");
  return composedLocationName.substr(0, pos);
}
