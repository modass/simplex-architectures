/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 23.06.22.
 */

#ifndef SIMPLEXARCHITECTURES_PUREPURSUITCONTROLLER_H
#define SIMPLEXARCHITECTURES_PUREPURSUITCONTROLLER_H

#include "AbstractController.h"
#include "../types.h"
#include "../utility/RaceTrack.h"
#include "../utility/coordinate_util.h"

namespace simplexArchitectures {

class PurePursuitController : public AbstractController<Point, Point> {
 public:
  Point generateInput( Point state );

  Point lastWaypoint;
  Point currentWaypoint;

 private:
  RaceTrack track;

  const double wheelbase = 1.0;

};

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_PUREPURSUITCONTROLLER_H
