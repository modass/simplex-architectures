/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 20.02.22.
 */

#include "Trainer.h"

#include <hypro/algorithms/reachability/Reach.h>
#include <hypro/parser/antlr4-flowstar/ParserWrapper.h>
#include <hypro/representations/GeometricObjectBase.h>
#include <hypro/util/plotting/Plotter.h>
#include "../utility/octreePlotting.h"

#include "../utility/reachTreeUtility.h"
#include "TrainingHeuristics.h"

namespace simplexArchitectures {

void Trainer::run() {
  auto [mAutomaton, reachSettings] = hypro::parseFlowstarFile<double>( mModelFileName );
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
    for ( auto i = 0; i < mAutomaton.getLocations().size(); ++i ) {
      // octree to store reachable sets, here fixed to [0,1] in each dimension
      mTrees.emplace( i, hypro::Hyperoctree<Number>( mStorageSettings.treeSplits, mStorageSettings.treeDepth,
                                                     mStorageSettings.treeContainer ) );
    }
  }
  // run iterations
  auto i = 0;
  while ( i++ < mTrainingSettings.iterations ) {
    runIteration( settings );
  }
}

void Trainer::plot( const std::string& outfilename ) {
  hypro::Plotter<Number>& plt = hypro::Plotter<Number>::getInstance();
  for ( const auto& [locationIndex, tree] : mTrees ) {
    std::size_t idx = locationIndex;
    plt.setFilename( outfilename + "_" + ( *std::next( mAutomaton.getLocations().begin(), idx ) )->getName() );
    plotOctree( tree, plt );
  }
}

locationConditionMap Trainer::generateInitialStates() const {
  switch ( mTrainingSettings.heuristics ) {
    case INITIAL_STATE_HEURISTICS::RANDOM: {
      auto initialStatesFunctor{ Random() };
      return initialStatesFunctor( mAutomaton, mTrainingSettings );
    }
    case INITIAL_STATE_HEURISTICS::GRID: {
      auto initialStatesFunctor{ Grid( mTrainingSettings.subdivision, mTrainingSettings ) };
      return initialStatesFunctor( mAutomaton, mTrainingSettings );
    }
    case INITIAL_STATE_HEURISTICS::GRID_COVER: {
      auto initialStatesFunctor{ GridCover( mTrainingSettings.subdivision, mTrainingSettings ) };
      return initialStatesFunctor( mAutomaton, mTrainingSettings );
    }
  }
}

void Trainer::updateOctree( const std::vector<hypro::ReachTreeNode<Representation>>& roots ) {
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
          while ( i < mAutomaton.getLocations().size() ) {
            if ( mAutomaton.getLocations().at( i ) == node.getLocation() ) {
              mTrees.at( i ).add( s.projectOn( mStorageSettings.projectionDimensions ) );
            }
            ++i;
          }
        }
      }
    }
  }
}

void Trainer::runIteration( const hypro::Settings& settings ) {
  auto roots = hypro::makeRoots<Representation>( mAutomaton );
  // obtain and set new initial states for training
  auto newInitialStates = generateInitialStates();
  mAutomaton.setInitialStates( newInitialStates );
  // analysis
  auto reacher = hypro::reachability::Reach<Representation>( mAutomaton, settings.fixedParameters(),
                                                             settings.strategy().front(), roots );

  auto result = reacher.computeForwardReachability();

  // post processing
  if ( result != hypro::REACHABILITY_RESULT::SAFE ) {
    std::cout << "System is initially not safe, need to deal with this." << std::endl;
  } else if ( !hasFixedPoint( roots ) ) {
    std::cout << "System has no fixed point initially, need to deal with this." << std::endl;
  } else {
    updateOctree( roots );
  }
}

}  // namespace simplexArchitectures