//
// Created by bmaderbacher on 07.09.22.
//
#include "generateCarBaseController.h"

namespace simplexArchitectures {

std::pair<double, double> crossingInterval( Point A, Point B, std::size_t theta_discretization ) {
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

} // namespace
