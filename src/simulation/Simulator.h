//
// Created by bmaderbacher on 28.02.22.
//

#include "../types.h"
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
        // Assumptions: Hidden state variables of the specification are always clocks, we keep only the minimum and maximum in case simulation allows several values
        
        Point getBaseControllerOutput();  // extract the base controller output from the current state

        bool simulateSafety(const Point& ctrlInput); // Simulation with output, returns if execution is safe or not.
        std::map<LocPtr, std::vector<Box>> potentialNextStates();

        void update(const Point& ctrlInput, const Point& nextObservation); // Update current state based on input and next observation

        hypro::HybridAutomaton<Number> &mAutomaton;
        hypro::Settings mSettings;
        double mCycleTime = 1.0;
        std::vector<hypro::ReachTreeNode<Representation>> roots;
        std::map<LocPtr, std::set<Point>> mLastStates;

    private:
        static void setCtrlValue(Point &state, const Point &ctrlInput);
    };

}

#endif //SIMPLEXARCHITECTURES_SIMULATOR_H
