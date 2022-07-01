/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 25.03.22.
 */

#include "BicycleBaseController.h"

namespace simplexArchitectures {

Point BicycleBaseController::generateInput( Point state ) {
  double delta    = 0;
  double velocity = 1.0;

  Point target = computeTarget( state );
  // lookahead-distance l = sqrt(x^2 + y^2)
  double l = sqrt( pow( target.at( 0 ), 2 ) + pow( target.at( 1 ), 2 ) );

  // Stop the car in case the safe spot has been found
  // TODO make this more realistic later
  // TODO what if we are already close to the current waypoint?
  if ( l < 0.5 ) {
    velocity = 0.0;
  }

  // compute controll according to https://arxiv.org/pdf/2107.05815.pdf Eq. 1
  // if the point lies ahead
  if ( target.at( 0 ) > 0 ) {
    delta = atan( ( 2 * wheelbase * target.at( 1 ) ) / pow( l, 2 ) );
  } else {
    throw std::domain_error( "The target lies behind the car - this is not implemented yet." );
  }

  std::cout << "Base controller output: delta = " << delta << ", velocity = " << velocity << std::endl;

  return Point( { delta, velocity } );
}

Point BicycleBaseController::computeTarget( Point state ) {
  // project position on track
  auto projectedPoint = projectPointForwardsOnLine( state.projectOn( { 0, 1 } ), *lastWaypoint, *currentWaypoint );
  std::cout << "Projected point: " << projectedPoint << std::endl;
  // transform point into car coordinate-system
  // 1 translation
  Point translatedCoordinates = projectedPoint - state.projectOn( { 0, 1 } );
  // 2 rotate by heading
  double                  negHeading        = -state.at( 2 );
  hypro::matrix_t<double> rotationClockwise = hypro::matrix_t<double>::Zero( 2, 2 );
  rotationClockwise << cos( negHeading ), sin( negHeading ), -sin( negHeading ), cos( negHeading );
  return translatedCoordinates.linearTransformation( rotationClockwise );
}

}  // namespace simplexArchitectures