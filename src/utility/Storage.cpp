/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 17.03.22.
 */

#include "Storage.h"

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
  if(set.dimension() != mStorageSettings.projectionDimensions.size()){
    mTrees[locationName].add(set.projectOn(mStorageSettings.projectionDimensions));
  } else {
    mTrees[locationName].add(set);
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
