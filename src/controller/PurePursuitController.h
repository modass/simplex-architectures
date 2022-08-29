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

  std::vector<Point>::const_iterator lastWaypoint;
  std::vector<Point>::const_iterator currentWaypoint;

  RaceTrack track;
  size_t thetaDiscretization;
  double cycleTime;
 private:

  const double wheelbase = 1.0;
  const double maxLookahead = 6.0;

};

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_PUREPURSUITCONTROLLER_H
