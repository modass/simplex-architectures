/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 23.02.22.
 */

#include "octreePlotting.h"

namespace simplexArchitectures {

void plotOctree( const hypro::Hyperoctree<double> &octree, hypro::Plotter<double> &plt, bool plotSets ) {
  auto fillSettings = plt.settings();
  fillSettings.fill = true;

  assert(!octree.getContainer().projectOn( plt.settings().dimensions ).vertices().empty());
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
          plt.addObject( set.projectOn(plt.settings().dimensions).vertices(), hypro::plotting::colors[hypro::plotting::blue], fillSettings );
        }
      }
    }
  }
}

void plotOctrees( const std::map<std::string, hypro::Hyperoctree<double>>& octrees, hypro::Plotter<double>& plt, bool plotSets) {
  for(const auto&[key,tree] : octrees) {
    plotOctree(tree,plt,plotSets);
  }
}

}  // namespace simplexArchitectures