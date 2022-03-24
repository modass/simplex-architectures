//
// Created by stefan on 22.03.22.
//

#ifndef SIMPLEXARCHITECTURES_UTIL_H
#define SIMPLEXARCHITECTURES_UTIL_H

#define _USE_MATH_DEFINES
#include <math.h>

namespace modelGenerator {

inline double DegreesToRadians( double degrees ) { return degrees * ( M_PI / 180 ); }

}  // namespace modelGenerator

#endif  // SIMPLEXARCHITECTURES_UTIL_H
