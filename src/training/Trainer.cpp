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
  hypro::ReachabilitySettings reachSettings;
  std::tie(mAutomaton, reachSettings) = hypro::parseFlowstarFile<double>( mModelFileName );
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
    for ( const auto loc : mAutomaton.getLocations() ) {
      // octree to store reachable sets, here fixed to [0,1] in each dimension
      mTrees.emplace( loc->getName(), hypro::Hyperoctree<Number>( mStorageSettings.treeSplits, mStorageSettings.treeDepth,
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
  plt.rSettings().xPlotInterval = carl::Interval<double>(0,1);
  plt.rSettings().yPlotInterval = carl::Interval<double>(0,1);
  plt.rSettings().overwriteFiles = true;
  for ( const auto& [locationName, tree] : mTrees ) {
    plt.setFilename( outfilename + "_" + locationName );
    plotOctree( tree, plt, true );
    plt.plot2d(hypro::PLOTTYPE::png, true);
    plt.clear();
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
          for(const auto locPtr : mAutomaton.getLocations()) {
            if(locPtr->getName() == node.getLocation()->getName()) {
              mTrees.at( locPtr->getName() ).add(s.projectOn( mStorageSettings.projectionDimensions ));
            }
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

  // set up callbacks which are used by hypro to access the octree
  std::function<bool(const Representation&,const hypro::Location<double>*)> callback = [this](const auto& set, const auto locptr){ return mTrees.at(locptr->getName()).contains(set);};
  hypro::ReachabilityCallbacks<Representation,hypro::Location<double>> callbackStructure{callback};
  reacher.setCallbacks(callbackStructure);

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