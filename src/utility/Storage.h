/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 17.03.22.
 */

#ifndef SIMPLEXARCHITECTURES_STORAGE_H
#define SIMPLEXARCHITECTURES_STORAGE_H

#include <map>
#include <hypro/datastructures/Hyperoctree.h>
#include <hypro/util/serialization/serialization.h>
#include "../types.h"
#include "StorageSettings.h"
#include "fileExists.h"
#include <spdlog/spdlog.h>
#include <cereal/cereal.hpp>
#include <cereal/archives/binary.hpp>
//#include <cereal/archives/xml.hpp>
#include <cereal/types/map.hpp>
#include <filesystem>

namespace  simplexArchitectures {

struct Storage {

  Storage(std::string filename, const StorageSettings& settings) :storagefilename(filename), mStorageSettings(settings) {
    if ( fileExists( storagefilename ) ) {
      {
        std::ifstream fs{ storagefilename, std::ios::binary };
        // std::ifstream              fs{ mFilename };
        cereal::BinaryInputArchive iarchive( fs );
        // cereal::XMLInputArchive iarchive( fs );
        StorageSettings temporaryStorageSettings;
        iarchive( temporaryStorageSettings, mTrees );
        if(temporaryStorageSettings != settings) {
          spdlog::warn("Attention, storage settings loaded from file differ from passed storage settings, use storage settings from file.");
        }
        mStorageSettings = temporaryStorageSettings;
      }
      spdlog::info( "Read {} octrees which store {} sets from file", mTrees.size(), this->size() );
      for ( const auto& [name, tree] : mTrees ) {
        spdlog::debug( "Have tree for location {} which stores {} sets", name, tree.size() );
      }
    }
  }

  ~Storage() {
    this->write();
  }

  void add(std::string locationName, const hypro::Box<Number>& set);

  bool isContained(std::string locationName, const hypro::Box<Number>& set) const;

  /**
   * Getter for the number of stored safe sets
   * @return Number of stored safe sets
   */
  std::size_t size() const;

  /// Writes the currently available safe sets to a plot file, creating a file for each location.
  void plot( const std::string& outfilename ) const;
  /// Creates a combined plot of the currently available safe sets
  void plotCombined( const std::string& outfilename, bool writeAndClear = true ) const;

  void write();

  std::string                                       storagefilename;
  StorageSettings                                   mStorageSettings;  ///< settings for storing reachability results
  std::map<std::string, hypro::Hyperoctree<Number>> mTrees;            ///< storage for safe sets
};

}

#endif  // SIMPLEXARCHITECTURES_STORAGE_H
