/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 23.02.22.
 */

#include "octreePlotting.h"

namespace simplexArchitectures {

void plotOctree( const hypro::Hyperoctree<double> &octree, hypro::Plotter<double> &plt ) {
  if ( octree.isCovered() ) {
    plt.addObject( octree.getContainer().projectOn( plt.settings().dimensions ).vertices(),
                   hypro::plotting::colors[hypro::plotting::green] );
  } else {
    if ( !octree.getChildren().empty() ) {
      for ( const auto &child : octree.getChildren() ) {
        plotOctree( child, plt );
      }
    } else {
      plt.addObject( octree.getContainer().projectOn( plt.settings().dimensions ).vertices(),
                     hypro::plotting::colors[hypro::plotting::red] );
    }
  }
}

}  // namespace simplexArchitectures