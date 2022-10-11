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

inline double normalizeAngle(double radians ) {
  double angle = radians;
  while (angle < 0) {
    angle += 2*M_PI;
  }
  while (angle >= 2*M_PI) {
    angle -= 2*M_PI;
  }
  return angle;
}

inline Point projectPointOnLine( const Point& input, const Point& startLine, const Point& endLine ) {
  // shift line into origin
  auto v = ( endLine - startLine ).rawCoordinates();
  auto p = ( input - startLine ).rawCoordinates();
  // compute projection
  auto proj = v * ( v.dot( p ) / v.dot( v ) );
  return Point{ proj + startLine.rawCoordinates() };
}

inline Point projectPointForwardsOnLine( const Point& input, const Point& startLine, const Point& endLine,
                                         double shiftingFactor = 0.8 ) {
  // shift line into origin
  auto v = ( endLine - startLine ).rawCoordinates();
  auto p = ( input - startLine ).rawCoordinates();
  // compute projection
  auto shiftVector = (v/norm(v))*shiftingFactor;
  auto proj = v * ( v.dot( p ) / v.dot( v ) ) + shiftVector;
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

// generate constraints for the theta interval that allows to cross AB from left to right.
inline std::pair<double, double> crossingInterval( Point A, Point B, std::size_t theta_discretization ) {
  auto v = B - A;
  double crossingAngle = normalizeAngle( atan2(v[1], v[0]) );
  double theta_increment = ( 2 * M_PI ) / double( theta_discretization );

  size_t bucketA = getThetaBucket(crossingAngle, theta_discretization);
  double representativeA = getRepresentativeForThetaBucket(bucketA, theta_discretization);
  double angleA;
  if (representativeA < crossingAngle) {
    angleA = representativeA + theta_increment/2; //include bucket
  } else {
    angleA = representativeA - theta_increment/2; //exclude bucket
  }

  double crossingAngleB = normalizeAngle(crossingAngle-M_PI);
  size_t bucketB = getThetaBucket(crossingAngleB, theta_discretization);
  double representativeB = getRepresentativeForThetaBucket(bucketB, theta_discretization);
  double angleB;
  if (representativeB > crossingAngleB) {
    angleB = representativeB - theta_increment/2; //include bucket
  } else {
    angleB = representativeB + theta_increment/2; //exclude bucket
  }

  return {angleB, angleA};
}

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_COORDINATE_UTIL_H
