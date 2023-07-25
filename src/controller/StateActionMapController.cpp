//
// Created by bmaderbacher on 24.07.23.
//

#include "StateActionMapController.h"

namespace simplexArchitectures {

Point StateActionMapController::generateInput( Point state ) {

  auto potentialStateLocations = getLocationsForState(state, automaton);
  auto potentialLocations = getLocationForTheta(state[2], theta_discretization, potentialStateLocations);
  if(potentialLocations.size() > 1) {
    spdlog::warn("State corresponds to multiple locations:");
    for(auto l : potentialLocations) {
      spdlog::warn(l->getName());
    }
  }
  if(potentialLocations.empty()) {
    spdlog::warn("State corresponds to no location.");
  }
  auto newThetaBucket = stateActionMap.getAction(potentialLocations[0]->getName(),state);

  if(!newThetaBucket.has_value()) {
    spdlog::warn("No action found!");
    return Point{state[2],0};
  }

  auto actionName = newThetaBucket.value().getName();
  if(actionName == "stop") {
    spdlog::warn("BC stop");
    exit(-42);
    return Point{state[2],0};
  } else {
    auto thetaBucket = stoul(actionName.substr(10));
    auto theta = getRepresentativeForThetaBucket(thetaBucket, theta_discretization);
    return Point{theta,velocity};
  }
}

}