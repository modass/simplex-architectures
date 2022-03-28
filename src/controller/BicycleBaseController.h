/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 25.03.22.
 */

#ifndef SIMPLEXARCHITECTURES_BICYCLEBASECONTROLLER_H
#define SIMPLEXARCHITECTURES_BICYCLEBASECONTROLLER_H

#include "AbstractController.h"
#include "../types.h"
#include "../utility/RaceTrack.h"
#include "../utility/coordinate_util.h"

namespace simplexArchitectures {

class BicycleBaseController : public AbstractController<Point, Point> {
 public:
  Point generateInput( Point state );

  Point computeTarget( Point state );

  Point lastWaypoint;
  Point currentWaypoint;

 private:
  RaceTrack track;

  const double wheelbase = 1.0;
};

}

#endif  // SIMPLEXARCHITECTURES_BICYCLEBASECONTROLLER_H
