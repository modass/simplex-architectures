/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 16.02.22.
 */

#ifndef SIMPLEXARCHITECTURES_CONTROLLERUTIL_H
#define SIMPLEXARCHITECTURES_CONTROLLERUTIL_H

#include <hypro/datastructures/reachability/ReachTreev2.h>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/bundled/ostream.h>

namespace simplexArchitectures {

template <typename Node>
void cutoffControllerJumps( Node* node ) {
  // spdlog::trace("Cutoff in node {}", *node);
  auto children = node->getChildren();
  for ( auto childIt = children.begin(); childIt != children.end(); ++childIt ) {
    // TODO we should not hardcode the tick dimension (3) here!
    if ( !( *childIt )->getTransition()->getReset().isIdentity( 3 ) ) {
      // spdlog::trace("Cut off children from child {}.", **childIt);
      (*childIt)->eraseChildren();
      ( *childIt )->getFlowpipe().clear();
    } else {
      // spdlog::trace("Cutoff-descend from {} to child {}", *node, **childIt);
      cutoffControllerJumps( *childIt );
    }
  }
}

}

#endif // SIMPLEXARCHITECTURES_CONTROLLERUTIL_H