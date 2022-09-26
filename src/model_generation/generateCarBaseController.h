//
// Created by bmaderbacher on 07.09.22.
//

#ifndef SIMPLEXARCHITECTURES_GENERATECARBASECONTROLLER_H
#define SIMPLEXARCHITECTURES_GENERATECARBASECONTROLLER_H

#include <hypro/datastructures/HybridAutomaton/HybridAutomaton.h>
#include <hypro/datastructures/HybridAutomaton/output/Flowstar.h>

#include "../controller/CarBaseController.h"
#include "../utility/RoadSegment.h"

namespace simplexArchitectures {

template<typename HybridAutomaton>
CarBaseController<HybridAutomaton> generateCarBaseController( std::size_t theta_discretization,
                                             size_t      maxTurn,  // in theta buckets
                                             double stopZoneWidth, double borderAngle,
                                             const std::vector<GeneralRoadSegment>& segments, double velocity );

template<typename Location>
void generateCrossingTransition( Location* origin, Location* target, Point borderA,
                                  Point borderB, std::size_t theta_discretization );

}  // namespace simplexArchitectures

#include "generateCarBaseController.tpp"

#endif  // SIMPLEXARCHITECTURES_GENERATECARBASECONTROLLER_H
