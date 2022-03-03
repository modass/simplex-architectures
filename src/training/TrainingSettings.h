/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 21.02.22.
 */

#ifndef SIMPLEXARCHITECTURES_TRAININGSETTINGS_H
#define SIMPLEXARCHITECTURES_TRAININGSETTINGS_H

#include <hypro/representations/GeometricObjectBase.h>

#include <vector>

namespace simplexArchitectures {

enum INITIAL_STATE_HEURISTICS { RANDOM = 1, GRID = 2, GRID_COVER = 3 };

struct TrainingSettings {
  std::size_t              iterations;
  INITIAL_STATE_HEURISTICS heuristics;
  std::vector<std::size_t> wideningDimensions;
  hypro::Box<Number>       samplingArea;
  std::vector<std::size_t> subdivision     = {};
  Number                   timeHorizon     = 100.0;
  std::size_t              jumpDepth       = 200;
  Number                   initialSetWidth = 0.1;
  bool fullCoverage = false;
};

struct StorageSettings {
  std::vector<std::size_t> projectionDimensions;
  hypro::Box<Number>       treeContainer;
  std::size_t              treeSplits = 2;
  std::size_t              treeDepth  = 4;

  template <typename Archive>
  void serialize( Archive& ar ) {
    ar( projectionDimensions, treeContainer, treeSplits, treeDepth );
  }
};

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_TRAININGSETTINGS_H
