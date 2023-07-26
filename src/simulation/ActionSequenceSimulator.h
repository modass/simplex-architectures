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
  ActionSequenceSimulator( Automaton& automaton, Automaton& simAutomaton, const hypro::Settings& s, const Storage& storage)
      : mAutomaton( automaton ), mSimulationAutomaton( simAutomaton ), mSettings( s ), mStorage( storage ) {}

  //TODO mSimulationAutomaton should be a copy of automaton i.e. it should have its own cache!

  hypro::TRIBOOL simulate( const std::vector<hypro::Label>& controllerActions, LocPtr initialLocation, const hypro::Condition<Number>& initialBox );
  int storageReachedAtTime();
  bool wasStorageReached();
  std::vector<std::pair<std::pair<LocPtr, Box>, hypro::Label>> getStateActionPairs();
  std::vector<std::pair<LocPtr, Box>> getReachStates();
  std::vector<std::pair<LocPtr, Box>> getAllStates();
  void clear();


  Automaton&                        mAutomaton;  ///< environment + specification model
  Automaton&                         mSimulationAutomaton;  ///< environment + specification model + trace //TODO This should be a deep copy of mAutomaton
  hypro::Settings                   mSettings;
  double                            mCycleTime = 0.1;
  Eigen::Index                      mCycleTimeDimension = 3;
  std::vector<ReachTreeNode>        mRoots;
  std::vector<std::vector<std::pair<LocPtr, Box>>> mTimeStepNodes;
  std::vector<std::vector<std::tuple<LocPtr, Box, hypro::Label>>> mPotentialActions;

 private:
  static std::string extractOriginalLocationName(const std::string& composedLocationName);

  const Storage& mStorage;
};

}

#endif  // SIMPLEXARCHITECTURES_ACTIONSEQUENCESIMULATOR_H
