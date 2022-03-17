/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 20.02.22.
 */

#include "Trainer.h"

#include <hypro/algorithms/reachability/Reach.h>
#include <hypro/parser/antlr4-flowstar/ParserWrapper.h>
#include <hypro/representations/GeometricObjectBase.h>
#include <hypro/util/plotting/FlowpipePlotting.h>
#include <hypro/util/plotting/Plotter.h>

#include "../utility/octreePlotting.h"
#include "../utility/reachTreeUtility.h"

namespace simplexArchitectures {

void Trainer::run() {
  hypro::ReachabilitySettings reachSettings;
  reachSettings.timeStep = hypro::tNumber(0.01);
  // reachability analysis settings
  auto settings                                                   = hypro::convert( reachSettings );
  settings.rStrategy().front().detectJumpFixedPoints              = true;
  settings.rStrategy().front().detectFixedPointsByCoverage        = true;
  settings.rStrategy().front().detectContinuousFixedPointsLocally = true;
  settings.rStrategy().front().numberSetsForContinuousCoverage    = 2;
  settings.rStrategy().front().detectZenoBehavior                 = true;
  settings.rFixedParameters().localTimeHorizon                    = mTrainingSettings.timeHorizon;
  settings.rFixedParameters().jumpDepth                           = mTrainingSettings.jumpDepth;
  settings.rStrategy().begin()->aggregation                       = hypro::AGG_SETTING::AGG;
  // set up initial states generators
  InitialStatesGenerator* generator = getInitialStatesGenerator();
  // override iterations in case full coverage is requested
  if ( mTrainingSettings.fullCoverage ) {
    mTrainingSettings.iterations = computeRequiredIterationsForFullCoverage();
  }
  // run iterations
  spdlog::info( "Train {} iterations", mTrainingSettings.iterations );
  auto i = 1;
  while ( i++ <= mTrainingSettings.iterations ) {
    spdlog::info( "Run training iteration {}/{}", i - 1, mTrainingSettings.iterations );
    runIteration( settings, *generator );
  }
  delete generator;
}

bool Trainer::run(hypro::Settings settings, const locationConditionMap& initialStates) {
  // set up initial states generators
  InitialStatesGenerator* generator = new Single(initialStates);
  // set fixedpoint detection in settings explicitly
  settings.rStrategy().front().detectJumpFixedPoints              = true;
  settings.rStrategy().front().detectFixedPointsByCoverage        = true;
  settings.rStrategy().front().detectContinuousFixedPointsLocally = true;
  settings.rStrategy().front().numberSetsForContinuousCoverage    = 2;
  settings.rFixedParameters().globalTimeHorizon = -1;
  settings.rStrategy().front().detectZenoBehavior                 = true;
  settings.rFixedParameters().localTimeHorizon = 100;
  settings.rFixedParameters().jumpDepth = 200;
  settings.rStrategy().front().aggregation = hypro::AGG_SETTING::AGG;
  // run single iteration
  auto res = runIteration( settings, *generator );
  delete generator;
  return res;
}

void Trainer::plot( const std::string& outfilename ) {
  hypro::Plotter<Number>& plt    = hypro::Plotter<Number>::getInstance();
  plt.rSettings().xPlotInterval  = carl::Interval<double>( 0, 1 );
  plt.rSettings().yPlotInterval  = carl::Interval<double>( 0, 1 );
  plt.rSettings().dimensions = std::vector<std::size_t>{0,1};
  plt.rSettings().overwriteFiles = true;
  for ( const auto& [locationName, tree] : mStorage.mTrees ) {
    spdlog::debug( "Plot tree for location {} which stores {} sets to file {}", locationName, tree.size(),
                   outfilename + "_" + locationName );
    plt.setFilename( outfilename + "_" + locationName );
    plotOctree( tree, plt, true );
    plt.plot2d( hypro::PLOTTYPE::png, true );
    plt.clear();
  }
}

void Trainer::plotCombined(const std::string& outfilename) {
  hypro::Plotter<Number>& plt    = hypro::Plotter<Number>::getInstance();
  plt.rSettings().xPlotInterval  = carl::Interval<double>( 0, 1 );
  plt.rSettings().yPlotInterval  = carl::Interval<double>( 0, 1 );
  plt.rSettings().overwriteFiles = true;
  plt.setFilename(outfilename);
  plotOctrees(mStorage.mTrees,plt,true);
  plt.plot2d( hypro::PLOTTYPE::png, true );
  plt.clear();
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
  // constraints for cycle-time equals zero, encodes t <= 0 && -t <= -0
  hypro::matrix_t<Number> constraints = hypro::matrix_t<Number>::Zero( 2, 5 );
  hypro::vector_t<Number> constants   = hypro::vector_t<Number>::Zero( 2 );
  constraints( 0, 4 )                 = 1;
  constraints( 1, 4 )                 = -1;
  for ( const auto& r : roots ) {
    for ( const auto& node : hypro::preorder( r ) ) {
      for ( const auto& s : node.getFlowpipe() ) {
        // only store segments which contain states where the cycle time is zero
        if ( s.satisfiesHalfspaces( constraints, constants ).first != hypro::CONTAINMENT::NO ) {
          mStorage.add(node.getLocation()->getName(), s);
        }
      }
    }
  }
}

bool Trainer::runIteration( const hypro::Settings& settings, InitialStatesGenerator& generator ) {
  // obtain and set new initial states for training
  auto newInitialStates = generator( mAutomaton, mTrainingSettings );
  auto initialBox =
      Representation( newInitialStates.begin()->second.getMatrix(), newInitialStates.begin()->second.getVector() );
  std::stringstream ss;
  ss << initialBox;
  if ( mStorage.isContained( newInitialStates.begin()->first->getName(), initialBox ) ) {
    spdlog::debug( "Initial set {} for location {} already contained in octrees, skip iteration.", ss.str(),
                   newInitialStates.begin()->first->getName() );
    return true;
  }
  spdlog::info( "Location {}, initial box {}.", newInitialStates.begin()->first->getName(), ss.str() );
  // std::cout << "Initial set: " << Representation(newInitialStates.begin()->second.getMatrix(),
  // newInitialStates.begin()->second.getVector()).projectOn({0,1}) << std::endl;
  mAutomaton.setInitialStates( newInitialStates );
  // create roots for the reachtree from new initial states
  auto roots = hypro::makeRoots<Representation>( mAutomaton );
  // analysis
  auto reacher = hypro::reachability::Reach<Representation>( mAutomaton, settings.fixedParameters(),
                                                             settings.strategy().front(), roots );
  // set up callbacks which are used by hypro to access the octree
  std::size_t                                                                  shortcuts = 0;
  std::function<bool( const Representation&, const hypro::Location<double>* )> callback =
      [this, &shortcuts]( const auto& set, const auto locptr ) {
        if ( mStorage.isContained( locptr->getName(), set ) ) {
          ++shortcuts;
          return true;
        }
        return false;
      };
  hypro::ReachabilityCallbacks<Representation, hypro::Location<double>> callbackStructure{ callback };
  reacher.setCallbacks( callbackStructure );
  // start reachability analysis
  spdlog::debug("Start reachability analysis");
  auto result = reacher.computeForwardReachability();
  if(shortcuts > 0) {
    spdlog::debug( "Found {} fixed points by exploiting existing results.", shortcuts );
  }
  spdlog::debug("Reachability analysis done");
  // plot results
  /*
  hypro::Plotter<Number>& plt = hypro::Plotter<Number>::getInstance();
  plt.rSettings().filename = "training_out";
  plt.rSettings().overwriteFiles = false;
  plt.clear();
  for(const auto& root : roots) {
    for(const auto& node : hypro::preorder(root)) {
      for(const auto& set : node.getFlowpipe()) {
        if(node.hasFixedPoint() == hypro::TRIBOOL::TRUE)
          plt.addObject(set.projectOn({0,1}).vertices(), hypro::plotting::colors[hypro::plotting::green]);
        else
          plt.addObject(set.projectOn({0,1}).vertices(), hypro::plotting::colors[hypro::plotting::orange]);
      }
    }
  }
  plt.plot2d(hypro::PLOTTYPE::png);
  */

  // post processing
  if ( result != hypro::REACHABILITY_RESULT::SAFE ) {
    spdlog::debug( "System is not safe." );
    return false;
  } else if ( !hasFixedPoint( roots ) ) {
    spdlog::debug( "System has no fixed point." );
    return false;
  } else {
    spdlog::debug( "Analysis complete & safe (unbounded time), update octrees." );
    updateOctree( roots );
    return true;
  }
}

InitialStatesGenerator* Trainer::getInitialStatesGenerator() const {
  switch ( mTrainingSettings.heuristics ) {
    case INITIAL_STATE_HEURISTICS::RANDOM: {
      return new Random();
    }
    case INITIAL_STATE_HEURISTICS::GRID: {
      return new Grid( mTrainingSettings.subdivision, mTrainingSettings );
    }
    case INITIAL_STATE_HEURISTICS::GRID_COVER: {
      return new GridCover( mTrainingSettings.subdivision, mTrainingSettings );
    }
  }
}

std::size_t Trainer::computeRequiredIterationsForFullCoverage() const {
  std::size_t cells = mTrainingSettings.subdivision.front();
  for ( std::size_t i = 1; i < mTrainingSettings.subdivision.size(); ++i ) {
    cells *= mTrainingSettings.subdivision[i];
  }
  return cells * mAutomaton.getLocations().size();
}

}  // namespace simplexArchitectures