//
// Created by stefan on 22.03.22.
//

#ifndef SIMPLEXARCHITECTURES_UTIL_H
#define SIMPLEXARCHITECTURES_UTIL_H

#define _USE_MATH_DEFINES
#include <math.h>

namespace modelGenerator {

inline double degreeToRadiance(double angle) {
  return (M_PI / 180) * angle;
}

inline double radianceToDegree(double angle) {
  return (180 / M_PI) * angle;
}
}  // namespace modelGenerator

#endif  // SIMPLEXARCHITECTURES_UTIL_H
