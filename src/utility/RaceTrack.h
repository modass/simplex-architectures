/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 25.03.22.
 */

#ifndef SIMPLEXARCHITECTURES_RACETRACK_H
#define SIMPLEXARCHITECTURES_RACETRACK_H

#include "../types.h"
#include <hypro/representations/GeometricObjectBase.h>

namespace simplexArchitectures {

struct RaceTrack {
  hypro::Box<double> playground;
  std::vector<hypro::Box<double>> obstacles;


};

}

#endif  // SIMPLEXARCHITECTURES_RACETRACK_H
