/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 23.06.22.
 */

#include "PurePursuitController.h"

namespace simplexArchitectures {

Point PurePursuitController::generateInput( Point state ) {
  double delta    = 0;
  double velocity = 1.0;

  // TODO: The positions of x,y,theta in state should not be hard coded!
  Point target = translateToCarCoordinates(*currentWaypoint, Point{state.at(0), state.at(1)}, state.at(2));
  //Point target = *currentWaypoint;
  // lookahead-distance l = sqrt(x^2 + y^2)
  double l = sqrt( pow( target.at( 0 ), 2 ) + pow( target.at( 1 ), 2 ) );

  // Stop the car in case the safe spot has been found
  // TODO make this more realistic later
  if ( l < 0.5 ) {
    lastWaypoint = currentWaypoint;
    if(currentWaypoint != track.waypoints.end()) {
      ++currentWaypoint;
    } else {
      currentWaypoint = track.waypoints.begin();
    }
  }

  // compute controll according to https://arxiv.org/pdf/2107.05815.pdf Eq. 1
  // if the point lies ahead
  if ( target.at( 0 ) > 0 ) {
    delta = atan( ( 2 * wheelbase * target.at( 1 ) ) / pow( l, 2 ) );
  } else {
    throw std::domain_error( "The target lies behind the car - this is not implemented yet." );
  }

  std::cout << "Pure pursuit controller output: delta = " << delta << ", velocity = " << velocity << std::endl;

  return Point( { delta, velocity } );
}

}  // namespace simplexArchitectures