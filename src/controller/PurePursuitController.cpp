/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 23.06.22.
 */

#include "PurePursuitController.h"

#include <spdlog/spdlog.h>

#include "tool_car/ctrlConversion.h"

namespace simplexArchitectures {

Point PurePursuitController::generateInput( Point state ) {
  double delta    = 0;

  // TODO: The positions of x,y,theta in state should not be hard coded!
  Point target = translateToCarCoordinates(*currentWaypoint, Point{state.at(0), state.at(1)}, state.at(2));
  //Point target = *currentWaypoint;
  // lookahead-distance l = sqrt(x^2 + y^2)
//  double l = sqrt( pow( target.at( 0 ), 2 ) + pow( target.at( 1 ), 2 ) );

  double distanceToTarget = sqrt( pow( target.at( 0 ), 2 ) + pow( target.at( 1 ), 2 ) );
  double l = std::min(maxLookahead, distanceToTarget);
  if ( distanceToTarget < 3.0 ) {
    lastWaypoint = currentWaypoint;
    if(currentWaypoint != (track.waypoints.end()-1)) {
      spdlog::debug("Switch to next waypoint");
      ++currentWaypoint;
    } else {
      currentWaypoint = track.waypoints.begin();
      spdlog::debug("Switch to first waypoint");
    }
  }

  // compute controll according to https://arxiv.org/pdf/2107.05815.pdf Eq. 1
  // if the point lies ahead
  if ( target.at( 0 ) > 0 ) {
    delta = atan( ( 2 * wheelbase * target.at( 1 ) ) / pow( l, 2 ) );
  } else {
    spdlog::debug("The target lies behind the car - turning around.");
    if (target.at(1) > 0) {
      delta = M_PI/3;
    } else {
      delta = -M_PI/3;
    }
  }


  // convert delta to theta
  auto& currentTheta = state[2];
  auto thetaprime = tan(delta) * velocity / wheelbase;
  // offset & rescaling (to time difference, scaling-factor is used to pick the theta at a certain point in time during the control cycle)
  auto thetaPrimeScaled = scalingFactor * cycleTime * thetaprime;

  //clip thetaPrimeScaled
  if(thetaPrimeScaled > maxAngleChange) {
    thetaPrimeScaled = maxAngleChange;
  }
  if(thetaPrimeScaled < -maxAngleChange) {
    thetaPrimeScaled = -maxAngleChange;
  }

  auto targetTheta = currentTheta + thetaPrimeScaled;
  auto theta =  normalizeAngle(targetTheta);

  return Point( { theta, this->velocity } );
}

}  // namespace simplexArchitectures