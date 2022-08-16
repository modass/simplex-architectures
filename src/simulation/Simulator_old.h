/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 16.02.22.
 */
#include "../types.h"
#include "../controller/Controller.h"
#include "../controller/ControllerUtil.h"

#include <hypro/types.h>
#include <hypro/datastructures/reachability/ReachTreev2.h>
#include <hypro/datastructures/reachability/TreeTraversal.h>
#include <hypro/datastructures/HybridAutomaton/HybridAutomaton.h>
#include <hypro/algorithms/reachability/Reach.h>
#include <iostream>
#include <random>

#ifndef SIMPLEXARCHITECTURES_SIMULATOR_H
#define SIMPLEXARCHITECTURES_SIMULATOR_H

namespace simplexArchitectures {

using Matrix = hypro::matrix_t<Number>;
using Vector = hypro::vector_t<Number>;

struct Simulator {
  // Assumptions: Hidden state variables of the specification are always cLocPtrks, we keep only the minimum and maximum in case simulation allows several values

  void pointify();

  bool simulate(bool updateBaseController);

  Controller &mBaseController;
  Controller &mAdvancedController;
  hypro::HybridAutomaton<Number> &mAutomaton;
  hypro::Settings mSettings;
  double mCycleTime = 1.0;
  std::vector<ReachTreeNode> roots;
  std::map<LocPtr, std::set<Point>> mLastStates;
};

}

#endif // SIMPLEXARCHITECTURES_SIMULATOR_H
