//
// Created by bmaderbacher on 21.07.23.
//

#include "../types.h"
#include "../utility/Storage.h"
#include "CarActionSequenceTemplate.h"
#include "ActionSequenceSimulator.h"
#include "../tool_car/ctrlConversion.h"

#ifndef SIMPLEXARCHITECTURES_CARREPAIREXPLORER_H
#define SIMPLEXARCHITECTURES_CARREPAIREXPLORER_H

namespace simplexArchitectures {

struct CarRepairExplorer {
  CarRepairExplorer( size_t thetaDiscretization, size_t maxTurn, Automaton& automaton, Automaton& simAutomaton, const hypro::Settings& s, Storage& storage )
      : mThetaDiscretization( thetaDiscretization ), mMaxTurn( maxTurn ), mStorage( storage ), mTemplate(thetaDiscretization, maxTurn), mSimulator(automaton, simAutomaton, s, storage) {}
  size_t mThetaDiscretization;
  size_t mMaxTurn;
  Storage& mStorage;
  CarActionSequenceTemplate mTemplate;
  ActionSequenceSimulator mSimulator;

  bool findRepairSequence(LocPtr initialLocation, const hypro::Condition<Number>& initialBox);

 private:
  bool semiExhaustiveSearch(LocPtr initialLocation, const hypro::Condition<Number>& initialBox);
};

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_CARREPAIREXPLORER_H
