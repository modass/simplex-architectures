//
// Created by stefan on 27.03.22.
//

#ifndef SIMPLEXARCHITECTURES_COORDINATE_UTIL_H
#define SIMPLEXARCHITECTURES_COORDINATE_UTIL_H

#include <math.h>

#include "../types.h"

namespace simplexArchitectures {

inline double DegreesToRadians( double degrees ) { return degrees * ( M_PI / 180 ); }

inline Point projectPointOnLine( const Point& input, const Point& startLine, const Point& endLine ) {
  // shift line into origin
  auto v = ( endLine - startLine ).rawCoordinates();
  auto p = ( input - startLine ).rawCoordinates();
  // compute projection
  auto proj = v * ( v.dot( p ) / v.dot( v ) );
  return Point{ proj + startLine.rawCoordinates() };
}

inline Point projectPointForwardsOnLine( const Point& input, const Point& startLine, const Point& endLine,
                                         double shiftingFactor = 1.5 ) {
  // shift line into origin
  auto v = ( endLine - startLine ).rawCoordinates();
  auto p = ( input - startLine ).rawCoordinates();
  // compute projection
  auto proj = v * ( v.dot( p ) / v.dot( v ) ) * shiftingFactor;
  return Point{ proj + startLine.rawCoordinates() };
}

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_COORDINATE_UTIL_H
