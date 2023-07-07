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
  PurePursuitController(double v, double wb, double mL, double sF, double mC) : velocity(v), wheelbase(wb), maxLookahead(mL), scalingFactor(sF), maxAngleChange(mC) {}

  Point generateInput( Point state );

  std::vector<Point>::const_iterator lastWaypoint;
  std::vector<Point>::const_iterator currentWaypoint;

  RaceTrack track;
  size_t thetaDiscretization;
  double cycleTime;

 private:
  const double velocity = 1.0;
  const double maxAngleChange = 0.18;
  const double wheelbase = 0.25;
  const double maxLookahead = 4.0; // TODO find intuitive description ;)
  const double scalingFactor = 0.65; // 0.65 some interference, 0.75 very little interference, 1.0 no interference

};

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_PUREPURSUITCONTROLLER_H
