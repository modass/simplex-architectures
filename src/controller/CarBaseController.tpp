//
// Created by bmaderbacher on 07.09.22.
//

#pragma once

#include "CarBaseController.h"

#include <spdlog/fmt/bundled/ostream.h>
#include <spdlog/spdlog.h>

namespace simplexArchitectures {

template<typename HybridAutomaton>
Point CarBaseController<HybridAutomaton>::generateInput( Point state ) {
  spdlog::trace( "Get Base Controller output for state {}", state );
  auto numberOfSegments = segments.size();

  // find the correct segment and zone
  std::size_t is                  = 0;
  std::size_t iz                  = 0;
  auto        segment             = segments[0];
  bool        foundZoneAndSegment = false;
  auto        projectedState      = state.projectOn( { 0, 1, 2 } );
  for ( ; is < numberOfSegments; ++is ) {
    segment = segments[is];
    if ( segment.getLeftZoneInvariant( stopZoneWidth ).contains( projectedState ) ) {
      iz                  = 0;
      foundZoneAndSegment = true;
      break;
    } else if ( segment.getCenterZoneInvariant( stopZoneWidth ).contains( projectedState ) ) {
      iz                  = 1;
      foundZoneAndSegment = true;
      break;
    } else if ( segment.getRightZoneInvariant( stopZoneWidth ).contains( projectedState ) ) {
      iz                  = 2;
      foundZoneAndSegment = true;
      break;
    }
  }

  if(!foundZoneAndSegment){
    spdlog::warn("No base controller output available for state {}.", state);
  }

  double segmentAngle = segment.getSegmentAngle();
  double targetTheta;
  switch ( iz ) {
    case 0: {
      targetTheta = normalizeAngle( segmentAngle - borderAngle );
      break;
    }
    case 1: {
      targetTheta = segmentAngle;
      break;
    }
    case 2: {
      targetTheta = normalizeAngle( segmentAngle + borderAngle );
      break;
    }

  }
  auto targetThetaBucket = getThetaBucket( targetTheta, theta_discretization );


  double theta_increment      = ( 2 * M_PI ) / double( theta_discretization );
  double theta_min = 0.0;
  double theta_max = theta_increment;

  size_t t =0;
  for (t = 0; t < theta_discretization; t++) {
    if(theta_min <= state[2] && state[2] <= theta_max) {
      break;
    }
    theta_min += theta_increment;
    theta_max += theta_increment;
  }

  auto differenceLeft = targetThetaBucket >= t ? targetThetaBucket - t : theta_discretization + targetThetaBucket - t;
  auto differenceRight = t >= targetThetaBucket ? t - targetThetaBucket : theta_discretization + t - targetThetaBucket;

  size_t newThetaBucket;

  auto maxStopDifference = 1; // theta_discretization/8;
  if (iz == 1 && (differenceLeft <= maxStopDifference || differenceRight <= maxStopDifference)) {
    spdlog::warn("Base controller stopped!");
    return Point{projectedState.at(2), 0};
  }

  if (differenceLeft == 0) {
    newThetaBucket = targetThetaBucket;
  } else if (differenceLeft <= differenceRight) {
    auto turn = std::min(maxTurn, differenceLeft);
    newThetaBucket = t + turn >= theta_discretization ? t + turn - theta_discretization : t + turn;
  } else if (differenceRight < differenceLeft) {
    auto turn = std::min(maxTurn, differenceRight);
    newThetaBucket = turn > t ? theta_discretization+t-turn : t - turn;
  }

  auto theta = getRepresentativeForThetaBucket(newThetaBucket, theta_discretization);

  spdlog::trace("Target angle {} (bucket {}), new theta {} (bucket {})",targetTheta, targetThetaBucket, theta, newThetaBucket);
  return Point{theta,velocity};
}

}  // namespace simplexArchitecture