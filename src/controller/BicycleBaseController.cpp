/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 25.03.22.
 */

#include "BicycleBaseController.h"
#include <spdlog/spdlog.h>

namespace simplexArchitectures {

Point BicycleBaseController::generateInput( Point state ) {
  double delta    = 0;
  double velocity = 1.0;

  Point target = computeTarget( state );

  // lookahead-distance l = sqrt(x^2 + y^2)
  double distanceToTarget = sqrt( pow( target.at( 0 ), 2 ) + pow( target.at( 1 ), 2 ) );
  double l = distanceToTarget; //std::min(2.0,distanceToTarget);
  // Stop the car in case the safe spot has been found

//  std::cout << "Target in car coordinates: ( " << target.at(0) << "; " << target.at(1) << ") ; distance to target = " << distanceToTarget << std::endl;

  auto currentPosition = state.projectOn({0,1});
  auto waypointVector = *currentWaypoint - currentPosition;
  auto angleToWaypoint = atan2(waypointVector.at(1), waypointVector.at(0));

//  std::cout << "target y distance: " <<  abs(target.at(1)) <<" ; angle to waypoint = " << angleToWaypoint-state.at(2) << std::endl;
  if ( abs(target.at(1)) < 0.15 && abs(angleToWaypoint - state.at(2)) < 0.2 ) {
    velocity = 0.0;
  }

  // compute controll according to https://arxiv.org/pdf/2107.05815.pdf Eq. 1
  // if the point lies ahead
  if ( target.at( 0 ) > 0 ) {
    delta = atan( ( 2 * wheelbase * target.at( 1 ) ) / pow( l, 2 ) );
  } else {
    spdlog::debug("The target lies behind the car - turning around.");
    if (target.at(1) > 0) {
      delta = M_PI/2;
    } else {
      delta = -M_PI/2;
    }
  }
//  std::cout << "Base controller output: delta = " << delta << ", velocity = " << velocity << std::endl;

  return Point( { delta, velocity } );
}

Point BicycleBaseController::computeTarget( Point state ) {
  // project position on track
  auto projectedPoint = projectPointForwardsOnLine( state.projectOn( { 0, 1 } ), *lastWaypoint, *currentWaypoint );
//  std::cout << "Projected point: " << projectedPoint << std::endl;
  return translateToCarCoordinates(projectedPoint, Point{state.at(0), state.at(1)}, state.at(2));;
//  // transform point into car coordinate-system
//  // 1 translation
//  Point translatedCoordinates = projectedPoint - state.projectOn( { 0, 1 } );
//  // 2 rotate by heading
//  double                  negHeading        = -state.at( 2 );
//  hypro::matrix_t<double> rotationClockwise = hypro::matrix_t<double>::Zero( 2, 2 );
//  rotationClockwise << cos( negHeading ), sin( negHeading ), -sin( negHeading ), cos( negHeading );
//  return translatedCoordinates.linearTransformation( rotationClockwise );
}

}  // namespace simplexArchitectures