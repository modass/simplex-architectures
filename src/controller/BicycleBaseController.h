/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 25.03.22.
 */

#ifndef SIMPLEXARCHITECTURES_BICYCLEBASECONTROLLER_H
#define SIMPLEXARCHITECTURES_BICYCLEBASECONTROLLER_H

#include "BaseController.h"
#include "../types.h"
#include "../utility/RaceTrack.h"
#include "../utility/coordinate_util.h"
#include "../utility/RoadSegment.h"

namespace simplexArchitectures {

struct BicycleBaseController : public BaseController<Automaton,Point, Point> {

  Point generateInput( Point state );

  double velocity = 1.0;
  std::size_t theta_discretization;
  std::size_t maxTurn;
  double stopZoneWidth;
  double borderAngle;
  std::vector<RoadSegment> segments;
};

}

#endif  // SIMPLEXARCHITECTURES_BICYCLEBASECONTROLLER_H
