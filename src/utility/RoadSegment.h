/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 25.08.22.
 */

#ifndef SIMPLEXARCHITECTURES_ROADSEGMENT_H
#define SIMPLEXARCHITECTURES_ROADSEGMENT_H

namespace simplexArchitectures {

enum RoadOrientation {LeftToRight, BottomToTop, RightToLeft, TopToBottom};

struct RoadSegment {
  double   x_min;
  double   y_min;
  double   x_max;
  double   y_max;
  RoadOrientation orientation;
};

}

#endif  // SIMPLEXARCHITECTURES_ROADSEGMENT_H
