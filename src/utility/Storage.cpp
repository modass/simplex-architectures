/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 17.03.22.
 */

#include "Storage.h"
#include <hypro/util/plotting/Plotter.h>
#include "../utility/octreePlotting.h"

namespace simplexArchitectures {

std::size_t Storage::size() const {
  std::size_t res = 0;
  std::for_each( std::begin( mTrees ), std::end( mTrees ), [&res]( const auto& tree ) { res += tree.second.size(); } );
  return res;
}

void Storage::add( std::string locationName, const hypro::Box<Number>& set ) {
  // initialize hyperoctree, if not yet existing
  if(mTrees.find(locationName) == std::end(mTrees)) {
    mTrees.emplace(std::make_pair(locationName, hypro::Hyperoctree<Number>(mStorageSettings.treeSplits, mStorageSettings.treeDepth, mStorageSettings.treeContainer)));
  }
  hypro::CONTAINMENT filterResult = hypro::CONTAINMENT::FULL;
  hypro::Box<Number> filteredSet = std::move(set);
  if(!mStorageSettings.filter.isTrue()) {
    std::tie(filterResult,filteredSet) = set.satisfiesHalfspaces(mStorageSettings.filter.getMatrix(), mStorageSettings.filter.getVector());
  }
  if(filterResult == hypro::CONTAINMENT::FULL || filterResult == hypro::CONTAINMENT::PARTIAL || filterResult == hypro::CONTAINMENT::YES) {
    if(set.dimension() != mStorageSettings.projectionDimensions.size()){
      mTrees[locationName].add(filteredSet.projectOn(mStorageSettings.projectionDimensions));
    } else {
      mTrees[locationName].add(filteredSet);
    }
  }
}

bool Storage::isContained( std::string locationName, const hypro::Box<Number>& set ) const {
  if(mTrees.find(locationName) == std::end(mTrees)) {
    return false;
  }
  if(set.dimension() != mStorageSettings.projectionDimensions.size()){
    return mTrees.at(locationName).contains(set.projectOn(mStorageSettings.projectionDimensions));
  } else {
    return mTrees.at(locationName).contains(set);
  }
}

void Storage::plot( const std::string& outfilename ) {
  hypro::Plotter<Number>& plt    = hypro::Plotter<Number>::getInstance();
  // Assumption: All trees have the same base container size (intervals & dimension)
  plt.rSettings().xPlotInterval  = mTrees.begin()->second.getContainer().interval(0);
  plt.rSettings().yPlotInterval  = mTrees.begin()->second.getContainer().interval(1);
  plt.rSettings().dimensions = std::vector<std::size_t>{0,1};
  plt.rSettings().overwriteFiles = true;
  for ( const auto& [locationName, tree] : mTrees ) {
    spdlog::debug( "Plot tree for location {} which stores {} sets to file {}", locationName, tree.size(),
                   outfilename + "_" + locationName );
    plt.setFilename( outfilename + "_" + locationName );
    plotOctree( tree, plt, true );
    plt.plot2d( hypro::PLOTTYPE::png, true );
    plt.clear();
  }
}

void Storage::plotCombined(const std::string& outfilename, bool writeAndClear) {
  hypro::Plotter<Number>& plt    = hypro::Plotter<Number>::getInstance();
  // Assumption: All trees have the same base container size (intervals & dimension)
  plt.rSettings().xPlotInterval  = mTrees.begin()->second.getContainer().interval(0);
  plt.rSettings().yPlotInterval  = mTrees.begin()->second.getContainer().interval(1);
  plt.rSettings().dimensions = std::vector<std::size_t>{0,1};
  plt.rSettings().overwriteFiles = true;
  plt.setFilename(outfilename);
  plotOctrees(mTrees,plt,true);
  if(writeAndClear) {
    plt.plot2d( hypro::PLOTTYPE::png, true );
    plt.clear();
  }
}

void Storage::write() {
  // write tree to file upon destruction
  spdlog::info( "Write {} octrees which store {} sets to file.", mTrees.size(), this->size() );
  // if the file exists, use it as a backup
  if ( fileExists( storagefilename ) ) {
    std::filesystem::rename( storagefilename, storagefilename + ".bak" );
  }
  std::ofstream fs{ storagefilename, std::ios::binary };
  {
    cereal::BinaryOutputArchive oarchive( fs );
    // cereal::XMLOutputArchive oarchive( fs );
    oarchive( mStorageSettings, mTrees );
  }
}

}
