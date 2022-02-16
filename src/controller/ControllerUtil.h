/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 16.02.22.
 */

#ifndef SIMPLEXARCHITECTURES_CONTROLLERUTIL_H
#define SIMPLEXARCHITECTURES_CONTROLLERUTIL_H

#include <hypro/datastructures/reachability/ReachTreev2.h>

namespace simplexArchitectures {

template <typename R>
void cutoffControllerJumps( hypro::ReachTreeNode<R>* node ) {
  auto children = node->getChildren();
  for ( auto childIt = children.begin(); childIt != children.end(); ++childIt ) {
    if ( !( *childIt )->getTransition()->getReset().getAffineReset().isIdentity( 4 ) ) {
      (*childIt)->eraseChildren();
      ( *childIt )->getFlowpipe().clear();
    } else {
      cutoffControllerJumps( *childIt );
    }
  }
}

}

#endif // SIMPLEXARCHITECTURES_CONTROLLERUTIL_H
