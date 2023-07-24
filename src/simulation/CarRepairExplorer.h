//
// Created by bmaderbacher on 21.07.23.
//

#include "../types.h"
#include "../utility/Storage.h"
#include "../utility/StateActionMap.h"
#include "CarActionSequenceTemplate.h"
#include "ActionSequenceSimulator.h"
#include "../tool_car/ctrlConversion.h"
#include "SamplingUtility.h"


#ifndef SIMPLEXARCHITECTURES_CARREPAIREXPLORER_H
#define SIMPLEXARCHITECTURES_CARREPAIREXPLORER_H

namespace simplexArchitectures {

struct CarRepairExplorer {
  CarRepairExplorer( size_t thetaDiscretization, size_t maxTurn, Automaton& automaton, Automaton& simAutomaton, const hypro::Settings& s, Storage& storage, StateActionMap<Box, hypro::Label>& stateActionMap )
      : mThetaDiscretization( thetaDiscretization ), mMaxTurn( maxTurn ), mStorage( storage ), mStateActionMap(stateActionMap), mTemplate(thetaDiscretization, maxTurn), mSimulator(automaton, simAutomaton, s, storage) {}
  size_t mThetaDiscretization;
  size_t mMaxTurn;
  Storage& mStorage;
  StateActionMap<Box, hypro::Label>& mStateActionMap;
  CarActionSequenceTemplate mTemplate;
  ActionSequenceSimulator mSimulator;
  Number mBloating = 0.5;
  std::vector<std::size_t> mBloatingDimensions = {0,1};


  bool findRepairSequence(LocPtr initialLocation, const Point initialState);

 private:
  bool semiExhaustiveSearch(LocPtr initialLocation, const hypro::Condition<Number>& initialBox);
};

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_CARREPAIREXPLORER_H
