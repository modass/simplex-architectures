//
// Created by bmaderbacher on 05.07.23.
//

#include "../types.h"
#include "../controller/ControllerUtil.h"
#include "../utility/Storage.h"
#include <hypro/types.h>
#include <hypro/datastructures/reachability/ReachTreev2.h>
#include <hypro/datastructures/reachability/TreeTraversal.h>
#include <hypro/datastructures/HybridAutomaton/HybridAutomaton.h>
#include <hypro/algorithms/reachability/Reach.h>
#include <iostream>
#include <random>
#include <spdlog/spdlog.h>

#ifndef SIMPLEXARCHITECTURES_ACTIONSEQUENCESIMULATOR_H
#define SIMPLEXARCHITECTURES_ACTIONSEQUENCESIMULATOR_H

namespace simplexArchitectures {

using Matrix = hypro::matrix_t<Number>;
using Vector = hypro::vector_t<Number>;

struct ActionSequenceSimulator {
  ActionSequenceSimulator( Automaton& automaton, const hypro::Settings& s, const Storage& storage)
      : mAutomaton( automaton ), mSettings( s ), mStorage( storage ) {
    mSimulationAutomaton = automaton;
  }

  //TODO mSimulationAutomaton should be a copy of automaton i.e. it should have its own cache!

  hypro::TRIBOOL simulate( const std::vector<hypro::Label>& controllerActions, LocPtr initialLocation, const hypro::Condition<Number>& initialBox );
  int storageReachedAtTime();
  bool wasStorageReached();
  std::vector<std::pair<std::pair<LocPtr, Box>, hypro::Label>> getStateActionPairs();


  Automaton&                        mAutomaton;  ///< environment + specification model
  Automaton                         mSimulationAutomaton;  ///< environment + specification model + trace
  hypro::Settings                   mSettings;
  std::vector<ReachTreeNode>        mRoots;
  std::vector<std::vector<std::pair<std::pair<LocPtr, Box>, hypro::Label>>> mTimeStepNodes;

 private:
  static std::string extractOriginalLocationName(const std::string& composedLocationName);

  const Storage& mStorage;
};

}

#endif  // SIMPLEXARCHITECTURES_ACTIONSEQUENCESIMULATOR_H
