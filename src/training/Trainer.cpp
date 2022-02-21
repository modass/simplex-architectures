/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 20.02.22.
 */

#include "Trainer.h"

#include <hypro/algorithms/reachability/Reach.h>
#include <hypro/parser/antlr4-flowstar/ParserWrapper.h>
#include <hypro/representations/GeometricObjectBase.h>

#include "../utility/reachTreeUtility.h"
#include "TrainingHeuristics.h"

namespace simplexArchitectures {

void Trainer::run() {
  auto [automaton, reachSettings] = hypro::parseFlowstarFile<double>( mModelFileName );
  // reachability analysis settings
  auto settings                                                   = hypro::convert( reachSettings );
  settings.rStrategy().front().detectJumpFixedPoints              = true;
  settings.rStrategy().front().detectFixedPointsByCoverage        = true;
  settings.rStrategy().front().detectContinuousFixedPointsLocally = true;
  settings.rStrategy().front().numberSetsForContinuousCoverage    = 2;
  settings.rFixedParameters().localTimeHorizon                    = mTrainingSettings.timeHorizon;
  settings.rFixedParameters().jumpDepth                           = mTrainingSettings.jumpDepth;
  settings.rStrategy().begin()->aggregation                       = hypro::AGG_SETTING::AGG;
  // initialize storage
  if ( mTrees.empty() ) {
    for ( auto i = 0; i < automaton.getLocations().size(); ++i ) {
      // octree to store reachable sets, here fixed to [0,1] in each dimension
      mTrees.emplace( i, hypro::Hyperoctree<Number>( mStorageSettings.treeSplits, mStorageSettings.treeDepth,
                                                     mStorageSettings.treeContainer ) );
    }
  }
  // run iterations
  auto i = 0;
  while ( i++ < mTrainingSettings.iterations ) {
    runIteration( automaton, settings );
  }
}

locationConditionMap Trainer::generateInitialStates( const hypro::HybridAutomaton<Number>& automaton ) const {
  switch ( mTrainingSettings.heuristics ) {
    case INITIAL_STATE_HEURISTICS::RANDOM: {
      auto initialStatesFunctor{ Random() };
      return initialStatesFunctor( automaton, mTrainingSettings );
    }
  }
}

void Trainer::updateOctree( const std::vector<hypro::ReachTreeNode<Representation>>& roots,
                            const hypro::HybridAutomaton<Number>&                    automaton ) {
  //  constraints for cycle-time equals zero, encodes t <= 0 && -t <= -0
  hypro::matrix_t<Number> constraints = hypro::matrix_t<Number>::Zero( 2, 5 );
  hypro::vector_t<Number> constants   = hypro::vector_t<Number>::Zero( 2 );
  constraints( 0, 4 )                 = 1;
  constraints( 1, 4 )                 = -1;

  for ( const auto& r : roots ) {
    for ( const auto& node : hypro::preorder( r ) ) {
      for ( const auto& s : node.getFlowpipe() ) {
        // only store segments which contain states where the cycle time is zero
        if ( s.satisfiesHalfspaces( constraints, constants ).first != hypro::CONTAINMENT::NO ) {
          // get location index
          std::size_t i = 0;
          while ( i < automaton.getLocations().size() ) {
            if ( automaton.getLocations().at( i ) == node.getLocation() ) {
              mTrees.at( i ).add( s.projectOn( mStorageSettings.projectionDimensions ) );
            }
            ++i;
          }
        }
      }
    }
  }
}

void Trainer::runIteration( hypro::HybridAutomaton<double> automaton, const hypro::Settings& settings ) {
  auto roots = hypro::makeRoots<Representation>( automaton );
  // obtain and set new initial states for training
  auto newInitialStates = generateInitialStates( automaton );
  automaton.setInitialStates( newInitialStates );
  // analysis
  auto reacher = hypro::reachability::Reach<Representation>( automaton, settings.fixedParameters(),
                                                             settings.strategy().front(), roots );
  std::cout << "Run initial analysis ... " << std::flush;
  auto result = reacher.computeForwardReachability();
  std::cout << "done, result: " << result << std::endl;
  // post processing
  if ( result != hypro::REACHABILITY_RESULT::SAFE ) {
    std::cout << "System is initially not safe, need to deal with this." << std::endl;
  } else if ( !hasFixedPoint( roots ) ) {
    std::cout << "System has no fixed point initially, need to deal with this." << std::endl;
  } else {
    updateOctree( roots, automaton );
  }
}

}  // namespace simplexArchitectures