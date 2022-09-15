/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 20.02.22.
 */

#ifndef SIMPLEXARCHITECTURES_TRAINER_H
#define SIMPLEXARCHITECTURES_TRAINER_H

#include <hypro/datastructures/Hyperoctree.h>
#include <hypro/datastructures/reachability/ReachTreev2.h>
#include <hypro/datastructures/reachability/Settings.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <fstream>

#include "../types.h"
#include "../utility/fileExists.h"
#include "../utility/Storage.h"
#include "../utility/StorageSettings.h"
#include "TrainingHeuristics.h"
#include "TrainingSettings.h"

namespace simplexArchitectures {

using locationConditionMap = Automaton::locationConditionMap;

/**
 * Class used for training with a basic controller to collect sets of safe
 * states. Allows to store and load already computed sets of safe states to a
 * file.
 */
class Trainer {
 public:
  Trainer( Automaton& automaton, TrainingSettings trainingSettings, Storage& storage )
      : mTrainingSettings( trainingSettings ), mStorage( storage ), mAutomaton( automaton ) {}
  ~Trainer() { mStorage.write(); }
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
  bool run(hypro::Settings settings, const locationConditionMap& initialStates);

 private:
  /// Calls the chosen generator and returns a new set of initial states (location + state set)
  locationConditionMap generateInitialStates() const;
  /**
   * Integrates the computed sets of reachable states stored in the reachability tree into the training storage
   * @param roots reachability tree
   */
  void updateOctree( const std::vector<ReachTreeNode>& roots );
  /**
   * Runs one training iteration, i.e., one run with one initial set and tries to find a fixed point
   * @param settings settings for the reachability analysis
   * @param generator generator which implements a strategy for training and creates novel initial sets
   */
  bool runIteration( const hypro::Settings& settings, InitialStatesGenerator& generator );
  /**
   * Getter for the initial state generator according to the heuristic specified in the training settings
   * @return pointer to a generator
   */
  InitialStatesGenerator* getInitialStatesGenerator() const;
  /// Helper-function which determines how many initial states need to be generated to achieve full coverage of the
  /// training area when using GRID_COVERAGE
  std::size_t computeRequiredIterationsForFullCoverage() const;

 protected:
  TrainingSettings mTrainingSettings;  ///< settings for training
  Storage&         mStorage;
  Automaton&       mAutomaton;  ///< automaton representing the model
};

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_TRAINER_H
