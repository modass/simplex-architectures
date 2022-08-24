//
// Created by bmaderbacher on 06.07.22.
//

#ifndef SIMPLEXARCHITECTURES_GENERATESIMPLEBASECONTROLLER_H
#define SIMPLEXARCHITECTURES_GENERATESIMPLEBASECONTROLLER_H

#include <hypro/datastructures/HybridAutomaton/HybridAutomaton.h>
#include <hypro/datastructures/HybridAutomaton/output/Flowstar.h>

namespace simplexArchitectures {

enum RoadOrientation {LeftToRight, BottomToTop, RightToLeft, TopToBottom};

struct RoadSegment {
  double   x_min;
  double   y_min;
  double   x_max;
  double   y_max;
  RoadOrientation orientation;
};

/**
 * Computes a hybrid automaton for a base controller
 * @return
 */
hypro::HybridAutomaton<double> generateSimpleBaseController(std::size_t theta_discretization,
                                                             double stopZoneWidth,
                                                             double centerZoneWidth,
                                                             double centerAngle,
                                                             double borderAngle,
                                                             const std::vector<RoadSegment>& segments);

}
#endif  // SIMPLEXARCHITECTURES_GENERATEBASECONTROLLER_H