/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 25.03.22.
 */

#include "BicycleBaseController.h"
#include <spdlog/spdlog.h>

namespace simplexArchitectures {

Point BicycleBaseController::generateInput( Point state ) {
  auto numberOfSegments = segments.size();
  // zones: borderLeft, centerLeft, stop, centerRight, borderRight
  // bucket indices: (segment, zone)
  std::map<std::tuple<std::size_t, std::size_t>, hypro::Location<double>*> buckets;
  std::map<std::tuple<std::size_t, std::size_t, std::size_t>, std::pair<double,double>> outputs;

  double theta_increment = ( 2 * M_PI ) / double( theta_discretization );
  double theta_representative  = theta_increment / 2.0;

  // find the correct segment and zone
  std::size_t is = 0;
  std::size_t iz = 0;
  auto        segment = segments[0];
  for ( ; is < numberOfSegments; ++is ) {
    segment         = segments[is];
    bool horizontal = segment.orientation == LeftToRight || segment.orientation == RightToLeft;

    for ( ; iz < 5; ++iz ) {
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
        break;
      }
    }
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
    double theta = segmentAngle;
    double velocity = 1;
    switch ( iz ) {
      case 0: {
        theta = normalizeAngle( segmentAngle - borderAngle );
        break;
      }
      case 1: {
        theta = normalizeAngle(segmentAngle - centerAngle);
        break;
      }
      case 2: {
        velocity = 0;
        break;
      }
      case 3: {
        theta = normalizeAngle( segmentAngle + centerAngle );
        break;
      }
      case 4: {
        theta = normalizeAngle( segmentAngle + borderAngle );
        break;
      }

    }
    return Point{theta,velocity};
}


}  // namespace simplexArchitectures