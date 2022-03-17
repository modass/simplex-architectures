/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 17.03.22.
 */

#ifndef SIMPLEXARCHITECTURES_STORAGESETTINGS_H
#define SIMPLEXARCHITECTURES_STORAGESETTINGS_H

#include <vector>
#include <hypro/representations/GeometricObjectBase.h>
#include "../types.h"

namespace simplexArchitectures {

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

inline bool operator==(const StorageSettings& s1, const StorageSettings& s2) {
  return s1.treeSplits == s2.treeSplits && s1.treeDepth == s2.treeDepth && s1.projectionDimensions == s2.projectionDimensions && s1.treeContainer == s2.treeContainer;
}

inline bool operator!=(const StorageSettings& s1, const StorageSettings& s2) {
  return !(s1==s2);
}

}

#endif  // SIMPLEXARCHITECTURES_STORAGESETTINGS_H
