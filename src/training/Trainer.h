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
    }
  }
  ~Trainer() {
    // write tree to file upon destruction
    std::ofstream               fs{ mFilename, std::ios::binary };
    cereal::BinaryOutputArchive oarchive( fs );
    oarchive( mTrees );
  }
  void run();

 private:
  locationConditionMap generateInitialStates( const hypro::HybridAutomaton<Number>& automaton ) const;
  void                 updateOctree( const std::vector<hypro::ReachTreeNode<Representation>>& roots,
                                     const hypro::HybridAutomaton<Number>&                    automaton );
  void                 runIteration( hypro::HybridAutomaton<double> automaton, const hypro::Settings& settings );

 protected:
  TrainingSettings mTrainingSettings;                        ///< settings for training
  StorageSettings  mStorageSettings;                         ///< settings for storing reachability results
  std::string      mFilename = "treearchive";                ///< filename for saving and loading safe sets
  std::string      mModelFileName;                           ///< filename of the model file used for training
  std::map<std::size_t, hypro::Hyperoctree<Number>> mTrees;  ///< storage for safe sets
};

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_TRAINER_H
