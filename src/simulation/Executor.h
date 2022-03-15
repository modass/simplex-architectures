//
// Created by bmaderbacher on 15.03.22.
//

#include "../types.h"
#include <hypro/types.h>
#include <hypro/datastructures/reachability/ReachTreev2.h>
#include <hypro/datastructures/reachability/TreeTraversal.h>
#include <hypro/datastructures/HybridAutomaton/HybridAutomaton.h>
#include <hypro/algorithms/reachability/Reach.h>
#include "../controller/ControllerUtil.h"
#include <random>

#ifndef SIMPLEXARCHITECTURES_EXECUTOR_H
#define SIMPLEXARCHITECTURES_EXECUTOR_H

namespace simplexArchitectures {

    using Matrix = hypro::matrix_t<Number>;
    using Vector = hypro::vector_t<Number>;

    struct Executor {
        Point execute(const Point& ctrlInput);
        hypro::HybridAutomaton<Number> &mAutomaton;
    private:
        static void setCtrlValue(Point &state, const Point &ctrlInput);
        hypro::Settings mSettings;
        double mCycleTime = 1.0;
        std::vector<hypro::ReachTreeNode<Representation>> roots;
        LocPtr mLastLocation;
        Point mLastState;
        std::mt19937 mGenerator;
    };

}
#endif //SIMPLEXARCHITECTURES_EXECUTOR_H
