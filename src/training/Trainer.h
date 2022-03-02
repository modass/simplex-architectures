/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 20.02.22.
 */

#ifndef SIMPLEXARCHITECTURES_TRAINER_H
#define SIMPLEXARCHITECTURES_TRAINER_H

#include <hypro/datastructures/Hyperoctree.h>
#include <hypro/datastructures/reachability/ReachTreev2.h>
#include <hypro/datastructures/reachability/Settings.h>

#include <cereal/archives/binary.hpp>
#include <cereal/types/map.hpp>
#include <fstream>
#include <spdlog/spdlog.h>

#include "../types.h"
#include "../utility/fileExists.h"
#include "TrainingSettings.h"

namespace simplexArchitectures {

using locationConditionMap = hypro::HybridAutomaton<double>::locationConditionMap;

/**
 * Class used for training with a basic controller to collect sets of safe
 * states. Allows to store and load already computed sets of safe states to a
 * file.
 */
class Trainer {
 public:
  Trainer( std::string outfilename, std::string modelfilename, TrainingSettings trainingSettings,
           StorageSettings storageSettings )
      : mFilename( outfilename ),
        mModelFileName( modelfilename ),
        mTrainingSettings( trainingSettings ),
        mStorageSettings( storageSettings ) {
    if ( fileExists( mFilename ) ) {
      {
        std::ifstream              fs{ mFilename, std::ios::binary };
        cereal::BinaryInputArchive iarchive( fs );
        iarchive( mTrees );
      }
      spdlog::info("Read {} octrees which store {} sets from file", mTrees.size(), this->size());
      for(const auto& [name,tree] : mTrees) {
        spdlog::debug("Have tree for location {} which stores {} sets", name, tree.size());
      }
    }
  }
  ~Trainer() {
    // write tree to file upon destruction
    spdlog::info("Write {} octrees which store {} sets to file.", mTrees.size(), this->size());
    std::ofstream               fs{ mFilename, std::ios::binary };
    cereal::BinaryOutputArchive oarchive( fs );
    oarchive( mTrees );
  }
  /**
   * Starts training with the provided settings. The trainer will try to load a file containing already discovered save
   * sets (as specified in the filename) and extend those.
   */
  void run();
  /// Writes the currently available safe sets to a plot file.
  void plot( const std::string& outfilename );

 private:
  locationConditionMap generateInitialStates() const;
  void                 updateOctree( const std::vector<hypro::ReachTreeNode<Representation>>& roots );
  void                 runIteration( const hypro::Settings& settings );

 protected:
  TrainingSettings mTrainingSettings;                            ///< settings for training
  StorageSettings  mStorageSettings;                             ///< settings for storing reachability results
  std::string      mFilename = "treearchive";                    ///< filename for saving and loading safe sets
  std::string      mModelFileName;                               ///< filename of the model file used for training
  std::map<std::string, hypro::Hyperoctree<Number>> mTrees;      ///< storage for safe sets
  hypro::HybridAutomaton<Number>                    mAutomaton;  ///< automaton representing the model
};

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_TRAINER_H
