/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 20.02.22.
 */

#ifndef SIMPLEXARCHITECTURES_TRAINER_H
#define SIMPLEXARCHITECTURES_TRAINER_H

#include <hypro/datastructures/Hyperoctree.h>
#include <hypro/datastructures/reachability/ReachTreev2.h>
#include <hypro/datastructures/reachability/Settings.h>

#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/types/map.hpp>
#include <fstream>
#include <spdlog/spdlog.h>
#include <filesystem>

#include "../types.h"
#include "../utility/fileExists.h"
#include "TrainingHeuristics.h"
#include "TrainingSettings.h"
#include "TrainingHeuristics.h"

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
        //std::ifstream              fs{ mFilename };
        cereal::BinaryInputArchive iarchive( fs );
        //cereal::XMLInputArchive iarchive( fs );
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
    // if the file exists, use it as a backup
    if( fileExists(mFilename)) {
      std::filesystem::rename(mFilename, mFilename +".bak");
    }
    std::ofstream               fs{ mFilename, std::ios::binary };
    //std::ofstream               fs{ mFilename };
    {
      cereal::BinaryOutputArchive oarchive( fs );
      //cereal::XMLOutputArchive oarchive( fs );
      oarchive( mTrees );
    }
  }
  /**
   * Starts training with the provided settings. The trainer will try to load a file containing already discovered save
   * sets (as specified in the filename) and extend those.
   */
  void run();
  /**
   * Train with the given settings, i.e., reachability analysis settings and initial state sets
   * @param settings reachability analysis settings
   * @param initialStates set of initial states
   */
  void run(const hypro::Settings& settings, const locationConditionMap& initialStates);
  /// Writes the currently available safe sets to a plot file, creating a file for each location.
  void plot( const std::string& outfilename );
  /// Creates a combined plot of the currently available safe sets
  void plotCombined(const std::string& outfilename);

 private:
  locationConditionMap generateInitialStates() const;
  void                 updateOctree( const std::vector<hypro::ReachTreeNode<Representation>>& roots );
  void                 runIteration( const hypro::Settings& settings, InitialStatesGenerator& generator );
  std::size_t size() const;
  InitialStatesGenerator* getInitialStatesGenerator() const;
  std::size_t computeRequiredIterationsForFullCoverage() const;

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
