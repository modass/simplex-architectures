//
// Created by bmaderbacher on 26.09.22.
//

#ifndef SIMPLEXARCHITECTURES_GENERATECARSPECIFICATION_H
#define SIMPLEXARCHITECTURES_GENERATECARSPECIFICATION_H


#include <hypro/datastructures/HybridAutomaton/HybridAutomaton.h>
#include <hypro/datastructures/HybridAutomaton/output/Flowstar.h>

#include "../utility/RoadSegment.h"
#include "generateCarBaseController.h"
#include "../utility/coordinate_util.h"

namespace simplexArchitectures {

template <typename HybridAutomaton>
HybridAutomaton generateCarSpecification( std::size_t theta_discretization,
                                          double warningZoneWidth, double maxIncursionTime,
                                          const std::vector<GeneralRoadSegment>& segments );

template<typename Location>
void generateWarningCrossingTransition( Location* origin, Location* target, Point borderA,
                                 Point borderB, std::size_t theta_discretization, bool resetClock );

hypro::Condition<Number> leftWarningInvariant(GeneralRoadSegment segment, double relativeWarningZoneWidth);
hypro::Condition<Number> centerWarningInvariant(GeneralRoadSegment segment, double relativeWarningZoneWidth);
hypro::Condition<Number> rightWarningInvariant(GeneralRoadSegment segment, double relativeWarningZoneWidth);

static hypro::Condition<Number> pointsToWarningCondition( Point pointA, Point pointB, Point pointC, Point pointD );


}

#include "generateCarSpecification.tpp"

#endif  // SIMPLEXARCHITECTURES_GENERATECARSPECIFICATION_H
