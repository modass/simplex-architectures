//
// Created by bmaderbacher on 26.09.22.
//

#include "generateCarSpecification.h"

namespace simplexArchitectures {

hypro::Condition<Number> leftWarningInvariant(GeneralRoadSegment segment, double relativeWarningZoneWidth){
  auto pointA = segment.getWarningLeftStartLeft(relativeWarningZoneWidth);
  auto pointB = segment.getWarningLeftEndLeft(relativeWarningZoneWidth);
  auto pointC = segment.getWarningLeftEndRight(relativeWarningZoneWidth);
  auto pointD = segment.getWarningLeftStartRight(relativeWarningZoneWidth);

  return pointsToWarningCondition(pointA, pointB, pointC, pointD);
}
hypro::Condition<Number> centerWarningInvariant(GeneralRoadSegment segment, double relativeWarningZoneWidth){
  auto pointA = segment.getWarningLeftStartRight(relativeWarningZoneWidth);
  auto pointB = segment.getWarningLeftEndRight(relativeWarningZoneWidth);
  auto pointC = segment.getWarningRightEndLeft(relativeWarningZoneWidth);
  auto pointD = segment.getWarningRightStartLeft(relativeWarningZoneWidth);


  return pointsToWarningCondition(pointA, pointB, pointC, pointD);
}
hypro::Condition<Number> rightWarningInvariant(GeneralRoadSegment segment, double relativeWarningZoneWidth){
  auto pointA = segment.getWarningRightStartLeft(relativeWarningZoneWidth);
  auto pointB = segment.getWarningRightEndLeft(relativeWarningZoneWidth);
  auto pointC = segment.getWarningRightEndRight(relativeWarningZoneWidth);
  auto pointD = segment.getWarningRightStartRight(relativeWarningZoneWidth);

  return pointsToWarningCondition(pointA, pointB, pointC, pointD);
}
hypro::Condition<Number> pointsToWarningCondition( Point pointA, Point pointB, Point pointC, Point pointD ) {

  constexpr Eigen::Index         x     = 0;  // global position x
  constexpr Eigen::Index         y     = 1;  // global position y
  constexpr Eigen::Index         theta = 2;  // global theta heading
  constexpr Eigen::Index         timer = 3;  // spec clock
  constexpr Eigen::Index         C     = 4;  // constants

  std::vector<std::string> variableNames{ "x", "y", "theta", "timer" };

  auto numVariables = variableNames.size();

  using Matrix = hypro::matrix_t<double>;
  using Vector = hypro::vector_t<double>;

  auto rows = 4;

  Matrix constraints  = Matrix::Zero( rows, numVariables );
  Vector constants    = Vector::Zero( rows );

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