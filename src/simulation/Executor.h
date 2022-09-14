//
// Created by bmaderbacher on 15.03.22.
//

#include <hypro/algorithms/reachability/Reach.h>
#include <hypro/datastructures/HybridAutomaton/HybridAutomaton.h>
#include <hypro/datastructures/reachability/ReachTreev2.h>
#include <hypro/datastructures/reachability/ReachTreev2Util.h>
#include <hypro/datastructures/reachability/TreeTraversal.h>
#include <hypro/types.h>
#include <hypro/util/plotting/Plotter.h>

#include <random>

#include "../controller/ControllerUtil.h"
#include "../types.h"

#ifndef SIMPLEXARCHITECTURES_EXECUTOR_H
#define SIMPLEXARCHITECTURES_EXECUTOR_H

namespace simplexArchitectures {

/// Holds case-study specific parameters relevant for execution of the system
struct ExecutionSettings {
  Eigen::Index              clock_dimension;           ///< the state space dimension where the cycle-clock is stored
  std::vector<Eigen::Index> control_input_dimensions;  ///< the state space dimensions which holds the control-input
};

using Matrix = hypro::matrix_t<Number>;
using Vector = hypro::vector_t<Number>;

template <typename Automaton>
struct Executor {
  Executor( Automaton& automaton, LocPtr initialLocation, Point initialValuation )
      : mAutomaton( automaton ), mLastLocation( initialLocation ), mLastState( initialValuation ) {}

  Point execute( const Point& ctrlInput );

  Automaton&        mAutomaton;
  LocPtr            mLastLocation;
  Point             mLastState;
  hypro::Settings   mSettings;
  ExecutionSettings mExecutionSettings;
  bool              mPlot = true;
  std::function<LocPtr( Point, LocPtr )>
      mLocationUpdate;  ///< lambda-function that is used to update a location based on a ctrl-input

 private:
  double                     mCycleTime = 1.0;
  std::vector<ReachTreeNode> roots;
  std::mt19937               mGenerator;
};

}  // namespace simplexArchitectures

#include "Executor.tpp"

#endif  // SIMPLEXARCHITECTURES_EXECUTOR_H
