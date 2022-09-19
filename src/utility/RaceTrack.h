/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 25.03.22.
 */

#ifndef SIMPLEXARCHITECTURES_RACETRACK_H
#define SIMPLEXARCHITECTURES_RACETRACK_H

#include <hypro/representations/GeometricObjectBase.h>
#include <hypro/util/plotting/Colors.h>

#include "../types.h"
#include "RoadSegment.h"

namespace simplexArchitectures {

struct RaceTrack {
  std::vector<Point>              waypoints;
  hypro::Box<double>              playground;
  std::vector<hypro::Condition<double>> obstacles;
  std::vector<GeneralRoadSegment> roadSegments;
  double                          safetyMargin     = 0.2;
  Number                          startFinishX     = 5.0;
  Number                          startFinishYlow  = 0.0;
  Number                          startFinishYhigh = 3.0;

  std::vector<hypro::Condition<double>> createSafetySpecification() const { return obstacles; };

  void addToPlotter( std::optional<Point> car = std::nullopt, size_t color = hypro::plotting::red );

#ifndef NDEBUG
  inline bool is_sane() const { return waypoints.size() > 1 && !playground.empty();}
#endif
};

}

#endif  // SIMPLEXARCHITECTURES_RACETRACK_H
