//
// Created by bmaderbacher on 15.03.22.
//

#include "../types.h"
#include <hypro/types.h>
#include <hypro/datastructures/reachability/ReachTreev2.h>
#include <hypro/datastructures/reachability/ReachTreev2Util.h>
#include <hypro/datastructures/reachability/TreeTraversal.h>
#include <hypro/datastructures/HybridAutomaton/HybridAutomaton.h>
#include <hypro/algorithms/reachability/Reach.h>
#include "../controller/ControllerUtil.h"
#include <random>

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

struct Executor {
  Executor( hypro::HybridAutomaton<Number>& automaton, LocPtr initialLocation, Point initialValuation )
      : mAutomaton( automaton ), mLastLocation( initialLocation ), mLastState( initialValuation ) {}

  Point                           execute( const Point& ctrlInput );
  hypro::HybridAutomaton<Number>& mAutomaton;
  LocPtr                          mLastLocation;
  Point mLastState;
  hypro::Settings                 mSettings;
  ExecutionSettings               mExecutionSettings;

 private:
        double mCycleTime = 1.0;
        std::vector<hypro::ReachTreeNode<Representation>> roots;
        std::mt19937 mGenerator;
    };

}
#endif //SIMPLEXARCHITECTURES_EXECUTOR_H
