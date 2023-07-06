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
  auto result = reacher.computeForwardReachability();

  std::vector<std::vector<std::pair<LocPtr, Box>>> timeStepNodes(controllerActions.size());
  mTimeStepNodes.clear();
  mTimeStepNodes.resize(controllerActions.size());


  // safe all states reachable after each time step
  for ( auto& r : mRoots ) {
    for ( auto& node : hypro::preorder( r ) ) {
      // This should capture all nodes that are reached by a sync transition or are root nodes
      if(node.getTransition() == nullptr || !node.getTransition()->getLabels().empty()){
        auto locationName = node.getLocation()->getName();
        for ( size_t k = 0; k < controllerActions.size(); ++k ) {
          auto traceK = "trace_"+std::to_string(k);
          if(locationName.find(traceK) != std::string::npos){
            mTimeStepNodes[k].emplace_back(std::make_pair(node.getLocation(), node.getInitialSet()), controllerActions[k]);
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
      auto contained = mStorage.isContained(p.first.first->getName(), p.first.second);
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
    auto stateActionPairs = mTimeStepNodes[t];
    for (auto p: stateActionPairs) {
      LocPtr originalLocation = mAutomaton.getLocation(extractOriginalLocationName(p.first.first->getName()));
      Box box = p.first.second;
      auto action = p.second;
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
