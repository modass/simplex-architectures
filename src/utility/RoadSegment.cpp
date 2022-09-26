//
// Created by bmaderbacher on 07.09.22.
//

#include "RoadSegment.h"



namespace simplexArchitectures {

hypro::Condition<Number> GeneralRoadSegment::getLeftZoneInvariant( double relativeCenterZoneWidth ) {
  auto pointA = startLeft;
  auto pointB = endLeft;
  auto pointC = getCenterEndLeft(relativeCenterZoneWidth);
  auto pointD = getCenterStartLeft(relativeCenterZoneWidth);

  return pointsToCondition(pointA, pointB, pointC, pointD);
}
hypro::Condition<Number> GeneralRoadSegment::getCenterZoneInvariant( double relativeCenterZoneWidth ) {
  auto pointA = getCenterStartLeft(relativeCenterZoneWidth);
  auto pointB = getCenterEndLeft(relativeCenterZoneWidth);
  auto pointC = getCenterEndRight(relativeCenterZoneWidth);
  auto pointD = getCenterStartRight(relativeCenterZoneWidth);

  return pointsToCondition(pointA, pointB, pointC, pointD);
}
hypro::Condition<Number> GeneralRoadSegment::getRightZoneInvariant( double relativeCenterZoneWidth ) {
  auto pointA = getCenterStartRight(relativeCenterZoneWidth);
  auto pointB = getCenterEndRight(relativeCenterZoneWidth);
  auto pointC = endRight;
  auto pointD = startRight;

  return pointsToCondition(pointA, pointB, pointC, pointD);
}




Number GeneralRoadSegment::getSegmentAngle() const {
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

Point GeneralRoadSegment::getCenterStartLeft(double relativeCenterZoneWidth) const {
  auto relativeBorderZoneWidth = (1.0-relativeCenterZoneWidth) * 0.5;
  return startLeft + (startRight - startLeft) * relativeBorderZoneWidth;
}

Point GeneralRoadSegment::getCenterStartRight(double relativeCenterZoneWidth) const {
  auto relativeBorderZoneWidth = (1.0-relativeCenterZoneWidth) * 0.5;
  return startRight + (startLeft - startRight) * relativeBorderZoneWidth;
}

Point GeneralRoadSegment::getCenterEndLeft(double relativeCenterZoneWidth) const {
  auto relativeBorderZoneWidth = (1.0-relativeCenterZoneWidth) * 0.5;
  return endLeft + (endRight - endLeft) * relativeBorderZoneWidth;
}

Point GeneralRoadSegment::getCenterEndRight(double relativeCenterZoneWidth) const {
  auto relativeBorderZoneWidth = (1.0-relativeCenterZoneWidth) * 0.5;
  return endRight + (endLeft - endRight) * relativeBorderZoneWidth;
}
Point GeneralRoadSegment::getWarningLeftStartLeft( double relativeWarningZoneWidth ) const {
  return startLeft + (startRight - startLeft) * -relativeWarningZoneWidth;
}
Point GeneralRoadSegment::getWarningLeftStartRight( double relativeWarningZoneWidth ) const {
  return startLeft + (startRight - startLeft) * relativeWarningZoneWidth;
}
Point GeneralRoadSegment::getWarningLeftEndLeft( double relativeWarningZoneWidth ) const {
  return endLeft + (endRight - endLeft) * -relativeWarningZoneWidth;
}
Point GeneralRoadSegment::getWarningLeftEndRight( double relativeWarningZoneWidth ) const {
  return endLeft + (endRight - endLeft) * relativeWarningZoneWidth;
}
Point GeneralRoadSegment::getWarningRightStartLeft( double relativeWarningZoneWidth ) const {
  return startRight + (startRight - startLeft) * -relativeWarningZoneWidth;
}
Point GeneralRoadSegment::getWarningRightStartRight( double relativeWarningZoneWidth ) const {
  return startRight + (startRight - startLeft) * relativeWarningZoneWidth;;
}
Point GeneralRoadSegment::getWarningRightEndLeft( double relativeWarningZoneWidth ) const {
  return endRight + (endRight - endLeft) * -relativeWarningZoneWidth;
}
Point GeneralRoadSegment::getWarningRightEndRight( double relativeWarningZoneWidth ) const {
  return endRight + (endRight - endLeft) * relativeWarningZoneWidth;
}

}