/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 25.08.22.
 */

#ifndef SIMPLEXARCHITECTURES_ROADSEGMENT_H
#define SIMPLEXARCHITECTURES_ROADSEGMENT_H

#include "types.h"
#include "../tool_car/ctrlConversion.h"
#include "coordinate_util.h"

namespace simplexArchitectures {

enum RoadOrientation {LeftToRight, BottomToTop, RightToLeft, TopToBottom};

struct RoadSegment {
  double   x_min;
  double   y_min;
  double   x_max;
  double   y_max;
  RoadOrientation orientation;
};

struct GeneralRoadSegment {
  Point startLeft;
  Point startRight;
  Point endLeft;
  Point endRight;

  static constexpr Eigen::Index  x{0};  // global position x
  static constexpr Eigen::Index  y{1};  // global position y
  static constexpr Eigen::Index  theta{2};  // global theta heading
  static constexpr Eigen::Index  C{3};  // constants
  static constexpr size_t        numVariables{3};

  hypro::Condition<Number> getLeftZoneInvariant(double relativeCenterZoneWidth);
  hypro::Condition<Number> getCenterZoneInvariant(double relativeCenterZoneWidth);
  hypro::Condition<Number> getRightZoneInvariant(double relativeCenterZoneWidth);
  Number getSegmentAngle();

  static hypro::Condition<Number> pointsToCondition( Point pointA, Point pointB, Point pointC, Point pointD );
};

}

#endif  // SIMPLEXARCHITECTURES_ROADSEGMENT_H
