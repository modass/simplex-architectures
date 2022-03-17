/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 21.02.22.
 */

#ifndef SIMPLEXARCHITECTURES_TRAININGSETTINGS_H
#define SIMPLEXARCHITECTURES_TRAININGSETTINGS_H

#include <hypro/representations/GeometricObjectBase.h>

#include <vector>

namespace simplexArchitectures {

enum INITIAL_STATE_HEURISTICS { RANDOM = 1, GRID = 2, GRID_COVER = 3, SINGLE = 4 };

struct TrainingSettings {
  std::size_t              iterations;
  INITIAL_STATE_HEURISTICS heuristics;
  std::vector<std::size_t> wideningDimensions;
  hypro::Box<Number>       samplingArea;
  std::vector<std::size_t> subdivision     = {};
  Number                   timeHorizon     = 200.0;
  std::size_t              jumpDepth       = 200;
  Number                   initialSetWidth = 0.1;
  bool fullCoverage = false;
};


}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_TRAININGSETTINGS_H
