/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 25.03.22.
 */

#include "BicycleBaseController.h"

#include <spdlog/fmt/bundled/ostream.h>
#include <spdlog/spdlog.h>

#include "tool_car/ctrlConversion.h"

namespace simplexArchitectures {

Point BicycleBaseController::generateInput( Point state ) {

  spdlog::trace("Get Base Controller output for state {}",state);
  auto numberOfSegments = segments.size();
  // zones: borderLeft, centerLeft, stop, centerRight, borderRight
  // bucket indices: (segment, zone)
  std::map<std::tuple<std::size_t, std::size_t>, hypro::Location<double>*> buckets;
  std::map<std::tuple<std::size_t, std::size_t, std::size_t>, std::pair<double,double>> outputs;

  // find the correct segment and zone
  std::size_t is = 0;
  std::size_t iz = 0;
  auto        segment = segments[0];
  bool foundZoneAndSegment = false;
  for ( ; is < numberOfSegments; ++is ) {
    segment         = segments[is];
    bool horizontal = segment.orientation == LeftToRight || segment.orientation == RightToLeft;

    for ( iz = 0 ; iz < 5; ++iz ) {
      double x_low;
      double y_low;
      double x_high;
      double y_high;

      if (horizontal){
        x_low  = segment.x_min;
        x_high = segment.x_max;
        auto mid = segment.y_min + (segment.y_max - segment.y_min)/2.0;
        if(iz == 2) {
          y_low  = mid - stopZoneWidth/2;
          y_high = mid + stopZoneWidth/2;
        } else if ((iz == 1 && segment.orientation == LeftToRight) || (iz == 3 && segment.orientation == RightToLeft)) {
          y_low  = mid + stopZoneWidth/2;
          y_high = mid + stopZoneWidth/2 + centerZoneWidth;
        } else if ((iz == 1 && segment.orientation == RightToLeft) || (iz == 3 && segment.orientation == LeftToRight)) {
          y_high  = mid - stopZoneWidth/2;
          y_low   = mid - stopZoneWidth/2 - centerZoneWidth;
        }  else if ((iz == 0 && segment.orientation == LeftToRight) || (iz == 4 && segment.orientation == RightToLeft)) {
          y_low  = mid + stopZoneWidth/2 + centerZoneWidth;
          y_high = segment.y_max;
        } else if ((iz == 0 && segment.orientation == RightToLeft) || (iz == 4 && segment.orientation == LeftToRight)) {
          y_high  =  mid - stopZoneWidth/2 - centerZoneWidth;
          y_low   = segment.y_min;
        }
      } else { //vertical
        y_low  = segment.y_min;
        y_high = segment.y_max;
        auto mid = segment.x_min + (segment.x_max - segment.x_min)/2.0;
        if(iz == 2) {
          x_low  = mid - stopZoneWidth/2;
          x_high = mid + stopZoneWidth/2;
        } else if ((iz == 3 && segment.orientation == BottomToTop) || (iz == 1 && segment.orientation == TopToBottom)) {
          x_low  = mid + stopZoneWidth/2;
          x_high = mid + stopZoneWidth/2 + centerZoneWidth;
        } else if ((iz == 3 && segment.orientation == TopToBottom) || (iz == 1 && segment.orientation == BottomToTop)) {
          x_high  = mid - stopZoneWidth/2;
          x_low   = mid - stopZoneWidth/2 - centerZoneWidth;
        }  else if ((iz == 4 && segment.orientation == BottomToTop) || (iz == 0 && segment.orientation == TopToBottom)) {
          x_low  = mid + stopZoneWidth/2 + centerZoneWidth;
          x_high = segment.x_max;
        } else if ((iz == 4 && segment.orientation == TopToBottom) || (iz == 0 && segment.orientation == BottomToTop)) {
          x_high  =  mid - stopZoneWidth/2 - centerZoneWidth;
          x_low   = segment.x_min;
        }
      }

      // check whether current combination matches the input point - if so, we have found the correct segment and zone
      if(state[0] >= x_low && state[0] <= x_high && state[1] >= y_low && state[1] <= y_high) {
        foundZoneAndSegment = true;
        break;
      }
    }
    if(foundZoneAndSegment) {
      break;
    }
  }

  if(!foundZoneAndSegment){
    spdlog::warn("No base controller output available for state {}.", state);
  }

    //deduce correct theta bucket
    double segmentAngle;
    switch (segment.orientation) {
      case LeftToRight:
        segmentAngle = 0.0;
        break;
      case RightToLeft:
        segmentAngle = M_PI;
        break;
      case BottomToTop:
        segmentAngle = 0.5*M_PI;
        break;
      case TopToBottom:
        segmentAngle = 1.5*M_PI;
        break;
    }
    double targetTheta = segmentAngle;
    double velocity = 1;
    switch ( iz ) {
      case 0: {
        targetTheta = normalizeAngle( segmentAngle - borderAngle );
        break;
      }
      case 1: {
        targetTheta = normalizeAngle(segmentAngle - centerAngle);
        break;
      }
      case 2: {
        velocity = 0;
        targetTheta = state[2];
        break;
      }
      case 3: {
        targetTheta = normalizeAngle( segmentAngle + centerAngle );
        break;
      }
      case 4: {
        targetTheta = normalizeAngle( segmentAngle + borderAngle );
        break;
      }

    }

    auto targetThetaBucket = getThetaBucket( targetTheta, theta_discretization );

    double theta_increment      = ( 2 * M_PI ) / double( theta_discretization );
    double theta_min = 0.0;
    double theta_max = theta_increment;

    size_t t =0;
    for (t = 0; t < theta_discretization; t++) {
      if(theta_min <= state[2] && state[2] <= theta_max) {
        break;
      }
      theta_min += theta_increment;
      theta_max += theta_increment;
    }

    auto differenceLeft = targetThetaBucket >= t ? targetThetaBucket - t : theta_discretization + targetThetaBucket - t;
    auto differenceRight = t >= targetThetaBucket ? t - targetThetaBucket : theta_discretization + t - targetThetaBucket;

    size_t newThetaBucket;
    if (differenceLeft == 0) {
      newThetaBucket = targetThetaBucket;
    } else if (differenceLeft <= differenceRight) {
      auto turn = std::min(maxTurn, differenceLeft);
      newThetaBucket = t + turn >= theta_discretization ? t + turn - theta_discretization : t + turn;
    } else if (differenceRight < differenceLeft) {
      auto turn = std::min(maxTurn, differenceRight);
      newThetaBucket = turn > t ? theta_discretization+t-turn : t - turn;
    }

    auto theta = getRepresentativeForThetaBucket(newThetaBucket, theta_discretization);

    return Point{theta,velocity};
}


}  // namespace simplexArchitectures