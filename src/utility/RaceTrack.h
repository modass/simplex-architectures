/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 25.03.22.
 */

#ifndef SIMPLEXARCHITECTURES_RACETRACK_H
#define SIMPLEXARCHITECTURES_RACETRACK_H

#include "../types.h"
#include <hypro/representations/GeometricObjectBase.h>

namespace simplexArchitectures {

struct RaceTrack {
  std::vector<Point>              waypoints;
  hypro::Box<double>              playground;
  std::vector<hypro::Box<double>> obstacles;
  double                          safetyMargin = 0.2;

  std::vector<hypro::Condition<double>> createSafetySpecification() const;

  void addToPlotter( std::optional<Point> car = std::nullopt );
};

}

#endif  // SIMPLEXARCHITECTURES_RACETRACK_H
