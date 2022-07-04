//
// Created by stefan on 27.03.22.
//

#ifndef SIMPLEXARCHITECTURES_COORDINATE_UTIL_H
#define SIMPLEXARCHITECTURES_COORDINATE_UTIL_H

#include <math.h>

#include "../types.h"

namespace simplexArchitectures {
using Matrix = hypro::matrix_t<Number>;
using Vector = hypro::vector_t<Number>;

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

inline Point translateToCarCoordinates(const Point& point, const Point& carPosition, double carHeading) {
  Matrix rotation = Matrix::Zero( 2, 2);
  rotation( 0, 0 ) = std::cos( carHeading );
  rotation( 0, 1 ) = std::sin( carHeading );
  rotation( 1, 0 ) = -std::sin( carHeading );
  rotation( 1, 1 ) = std::cos( carHeading );

  Vector translatedPoint = (point - carPosition).rawCoordinates();
  Vector rotatedPoint = rotation * translatedPoint;
  return Point{rotatedPoint};
}

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_COORDINATE_UTIL_H
