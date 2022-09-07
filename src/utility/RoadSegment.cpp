//
// Created by bmaderbacher on 07.09.22.
//

#include "RoadSegment.h"



namespace simplexArchitectures {

hypro::Condition<Number> GeneralRoadSegment::getLeftZoneInvariant( double relativeCenterZoneWidth ) {
  auto relativeBorderZoneWidth = (1.0-relativeCenterZoneWidth) * 0.5;

  auto pointA = startLeft;
  auto pointB = endLeft;
  auto pointC = endLeft + (endRight - endLeft) * relativeBorderZoneWidth;
  auto pointD = startLeft + (startRight - startLeft) * relativeBorderZoneWidth;

  return pointsToCondition(pointA, pointB, pointC, pointD);
}
hypro::Condition<Number> GeneralRoadSegment::getCenterZoneInvariant( double relativeCenterZoneWidth ) {
  auto relativeBorderZoneWidth = (1.0-relativeCenterZoneWidth) * 0.5;

  auto pointA = startLeft + (startRight - startLeft) * relativeBorderZoneWidth;
  auto pointB = endLeft + (endRight - endLeft) * relativeBorderZoneWidth;
  auto pointC = endRight + (endLeft - endRight) * relativeBorderZoneWidth;
  auto pointD = startRight + (startLeft - startRight) * relativeBorderZoneWidth;

  return pointsToCondition(pointA, pointB, pointC, pointD);
}
hypro::Condition<Number> GeneralRoadSegment::getRightZoneInvariant( double relativeCenterZoneWidth ) {
  auto relativeBorderZoneWidth = (1.0-relativeCenterZoneWidth) * 0.5;

  auto pointA = startRight + (startLeft - startRight) * relativeBorderZoneWidth;
  auto pointB = endRight + (endLeft - endRight) * relativeBorderZoneWidth;
  auto pointC = endRight;
  auto pointD = startRight;

  return pointsToCondition(pointA, pointB, pointC, pointD);
}

Number GeneralRoadSegment::getSegmentAngle() {
  auto pointS = startLeft + (startRight - startLeft) * 0.5;
  auto pointE = endLeft + (endRight - endLeft) * 0.5;
  auto headingVector = pointE - pointS;

  auto angle = atan2(headingVector[1], headingVector[0]);
  return normalizeAngle(angle);
}

hypro::Condition<Number> GeneralRoadSegment::pointsToCondition(Point pointA, Point pointB, Point pointC, Point pointD) {
  using Matrix = hypro::matrix_t<double>;
  using Vector = hypro::vector_t<double>;

  Matrix constraints  = Matrix::Zero( 4, numVariables );
  Vector constants    = Vector::Zero( 4 );

  constraints( 0, x ) = pointA[y] - pointB[y];
  constraints( 0, y ) = pointB[x] - pointA[x];
  constants( 0 )      = pointB[x] * pointA[y] - pointA[x] * pointB[y];

  constraints( 1, x ) = pointB[y] - pointC[y];
  constraints( 1, y ) = pointC[x] - pointB[x];
  constants( 1 )      = pointC[x] * pointB[y] - pointB[x] * pointC[y];

  constraints( 2, x ) = pointC[y] - pointD[y];
  constraints( 2, y ) = pointD[x] - pointC[x];
  constants( 2 )      = pointD[x] * pointC[y] - pointC[x] * pointD[y];

  constraints( 3, x ) = pointD[y] - pointA[y];
  constraints( 3, y ) = pointA[x] - pointD[x];
  constants( 3 )      = pointA[x] * pointD[y] - pointD[x] * pointA[y];

  return {constraints, constants};
}


}