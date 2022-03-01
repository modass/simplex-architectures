/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 23.02.22.
 */

#include "octreePlotting.h"

namespace simplexArchitectures {

void plotOctree( const hypro::Hyperoctree<double> &octree, hypro::Plotter<double> &plt, bool plotSets ) {
  spdlog::debug("Plot octree: {}", octree);
  if ( octree.isCovered() ) {
    plt.addObject( octree.getContainer().projectOn( plt.settings().dimensions ).vertices(),
                   hypro::plotting::colors[hypro::plotting::green] );
  } else {
    plt.addObject( octree.getContainer().projectOn( plt.settings().dimensions ).vertices(),
                   hypro::plotting::colors[hypro::plotting::red] );
    if ( !octree.getChildren().empty() ) {
      for ( const auto &child : octree.getChildren() ) {
        plotOctree( child, plt, plotSets );
      }
    } else {
      if(plotSets) {
        for(const auto& set : octree.getData()) {
          plt.addObject( set.projectOn(plt.settings().dimensions).vertices());
        }
      }
    }
  }
}

}  // namespace simplexArchitectures