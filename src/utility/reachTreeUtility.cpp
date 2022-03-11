/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 11.03.22.
 */
#include "reachTreeUtility.h"

namespace simplexArchitectures {

bool hasFixedPoint( const std::vector<hypro::ReachTreeNode<Representation>> &roots ) {
  for ( const auto &r : roots ) {
    for ( const auto &node : hypro::preorder( r ) ) {
      if ( node.hasFixedPoint() != hypro::TRIBOOL::TRUE ) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace simplexArchitectures
