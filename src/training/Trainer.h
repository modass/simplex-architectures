/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 20.02.22.
 */

#ifndef SIMPLEXARCHITECTURES_TRAINER_H
#define SIMPLEXARCHITECTURES_TRAINER_H

#include <hypro/datastructures/Hyperoctree.h>
#include <hypro/datastructures/reachability/ReachTreev2.h>
#include <hypro/datastructures/reachability/Settings.h>
#include <spdlog/spdlog.h>

#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/types/map.hpp>
#include <filesystem>
#include <fstream>

#include "../types.h"
#include "../utility/fileExists.h"
#include "TrainingHeuristics.h"
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
        std::ifstream fs{ mFilename, std::ios::binary };
        // std::ifstream              fs{ mFilename };
        cereal::BinaryInputArchive iarchive( fs );
        // cereal::XMLInputArchive iarchive( fs );
        iarchive( mStorageSettings, mTrees );
      }
      spdlog::info( "Read {} octrees which store {} sets from file", mTrees.size(), this->size() );
      for ( const auto& [name, tree] : mTrees ) {
        spdlog::debug( "Have tree for location {} which stores {} sets", name, tree.size() );
      }
    }
  }
  ~Trainer() {
    // write tree to file upon destruction
    spdlog::info( "Write {} octrees which store {} sets to file.", mTrees.size(), this->size() );
    // if the file exists, use it as a backup
    if ( fileExists( mFilename ) ) {
      std::filesystem::rename( mFilename, mFilename + ".bak" );
    }
    std::ofstream fs{ mFilename, std::ios::binary };
    // std::ofstream               fs{ mFilename };
    {
      cereal::BinaryOutputArchive oarchive( fs );
      // cereal::XMLOutputArchive oarchive( fs );
      oarchive( mStorageSettings, mTrees );
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
  void plotCombined( const std::string& outfilename );
  /// Getter for the training results
  const std::map<std::string, hypro::Hyperoctree<Number>>& getTrainingData() const { return mTrees; }

 private:
  /// Calls the chosen generator and returns a new set of initial states (location + state set)
  locationConditionMap generateInitialStates() const;
  /**
   * Integrates the computed sets of reachable states stored in the reachability tree into the training storage
   * @param roots reachability tree
   */
  void updateOctree( const std::vector<hypro::ReachTreeNode<Representation>>& roots );
  /**
   * Runs one training iteration, i.e., one run with one initial set and tries to find a fixed point
   * @param settings settings for the reachability analysis
   * @param generator generator which implements a strategy for training and creates novel initial sets
   */
  void runIteration( const hypro::Settings& settings, InitialStatesGenerator& generator );
  /**
   * Getter for the number of stored safe sets
   * @return Number of stored safe sets
   */
  std::size_t size() const;
  /**
   * Getter for the initial state generator according to the heuristic specified in the training settings
   * @return pointer to a generator
   */
  InitialStatesGenerator* getInitialStatesGenerator() const;
  /// Helper-function which determines how many initial states need to be generated to achieve full coverage of the
  /// training area when using GRID_COVERAGE
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
