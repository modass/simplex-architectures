//
// Created by bmaderbacher on 06.07.22.
//

#ifndef SIMPLEXARCHITECTURES_GENERATEBASECONTROLLER_H
#define SIMPLEXARCHITECTURES_GENERATEBASECONTROLLER_H

#include <hypro/datastructures/HybridAutomaton/HybridAutomaton.h>
#include <hypro/datastructures/HybridAutomaton/output/Flowstar.h>

#include "controller/BicycleBaseController.h"

namespace simplexArchitectures {
/**
 * Computes a hybrid automaton based on a discretization of tan(\delta), cos(theta), sin(theta)
 * @param delta_ranges
 * @param discretization
 * @return
 */
hypro::HybridAutomaton<double> generateBaseController( double x_min,
                                                       double x_max,
                                                       double y_min,
                                                       double y_max,
                                                       BicycleBaseController &ctrl,
                                                       std::pair<double, double> delta_ranges   = { -60, 60 },
                                                       std::size_t delta_discretization = 7,
                                                       std::size_t theta_discretization = 12,
                                                       double x_interval_size = 0.5,
                                                       double y_interval_size = 0.5
                                                       );

}
#endif  // SIMPLEXARCHITECTURES_GENERATEBASECONTROLLER_H
