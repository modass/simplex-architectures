//
// Created by bmaderbacher on 07.09.22.
//

#ifndef SIMPLEXARCHITECTURES_GENERATECARBASECONTROLLER_H
#define SIMPLEXARCHITECTURES_GENERATECARBASECONTROLLER_H

#include <hypro/datastructures/HybridAutomaton/HybridAutomaton.h>
#include <hypro/datastructures/HybridAutomaton/output/Flowstar.h>
#include "../utility/RoadSegment.h"
#include "../controller/CarBaseController.h"

namespace simplexArchitectures {

CarBaseController generateCarBaseController( std::size_t theta_discretization,
                                   size_t      maxTurn,  // in theta buckets
                                   double stopZoneWidth, double borderAngle,
                                   const std::vector<GeneralRoadSegment>& segments, double velocity );
}

#endif  // SIMPLEXARCHITECTURES_GENERATECARBASECONTROLLER_H
