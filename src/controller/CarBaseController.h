//
// Created by bmaderbacher on 07.09.22.
//

#ifndef SIMPLEXARCHITECTURES_CARBASECONTROLLER_H
#define SIMPLEXARCHITECTURES_CARBASECONTROLLER_H

#include "../types.h"
#include "../utility/RaceTrack.h"
#include "../utility/RoadSegment.h"
#include "../utility/coordinate_util.h"
#include "BaseController.h"

#include <utility>

namespace simplexArchitectures {

template <typename HybridAutomaton>
class CarBaseController : public BaseController<HybridAutomaton, Point, Point> {
 public:
  CarBaseController( double v, size_t mT, double sZW, double bA, size_t tD )
      : velocity( v ), maxTurn( mT ), theta_discretization( tD ), stopZoneWidth(sZW), borderAngle(bA) {}

  Point generateInput( Point state ) override;

  double                   velocity = 1.0;
  std::size_t              theta_discretization;
  std::size_t              maxTurn;
  double                   stopZoneWidth;
  double                   borderAngle;
  std::vector<GeneralRoadSegment> segments;
};

}  // namespace simplexArchitectures

#include "CarBaseController.tpp"

#endif  // SIMPLEXARCHITECTURES_CARBASECONTROLLER_H
