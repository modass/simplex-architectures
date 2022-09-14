//
// Created by bmaderbacher on 02.09.22.
//

#include "RandomCarController.h"

#include <spdlog/spdlog.h>

#include "tool_car/ctrlConversion.h"
namespace simplexArchitectures {

Point RandomCarController::generateInput( Point state ) {

  double theta_increment      = ( 2 * M_PI ) / double( theta_discretization );
  double theta_min = 0.0;
  double theta_max = theta_increment;

  size_t t =0;
  for (t = 0; t < theta_discretization; t++) {
    if(theta_min <= state[2] && state[2] <= theta_max) {
      break;
    }
    theta_min += theta_increment;
    theta_max += theta_increment;
  }

  auto turn = distrib(gen);
  auto newThetaBucket = int(t) + turn;
  if (newThetaBucket < 0) {
    newThetaBucket = newThetaBucket + theta_discretization;
  }
  if (newThetaBucket >= theta_discretization) {
    newThetaBucket = newThetaBucket - theta_discretization;
  }

  auto theta = getRepresentativeForThetaBucket(newThetaBucket, theta_discretization);
  return Point{theta,velocity};
}

}