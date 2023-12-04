/*
 * Created by Stefan
         Schupp <stefan.schupp@tuwien.ac.at> on 20.02.22.
 */

#include "Trainer.h"

#include <hypro/algorithms/reachability/Reach.h>
#include <hypro/parser/antlr4-flowstar/ParserWrapper.h>
#include <hypro/representations/GeometricObjectBase.h>

#include "../utility/reachTreeUtility.h"

namespace simplexArchitectures {

void Trainer::run() {
  hypro::ReachabilitySettings reachSettings;
  reachSettings.timeStep = hypro::tNumber(0.01);
  // reachability analysis settings
  auto settings                                                   = hypro::convert( reachSettings );
  settings.rStrategy().front().detectJumpFixedPoints              = true;
  settings.rStrategy().front().detectFixedPointsByCoverage        = true;
  settings.rStrategy().front().detectContinuousFixedPointsLocally = false; // true
  settings.rStrategy().front().numberSetsForContinuousCoverage    = 0;     // 2
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
  settings.rStrategy().front().detectContinuousFixedPointsLocally = false; // true
  settings.rStrategy().front().numberSetsForContinuousCoverage    = 0; // 2
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

void Trainer::updateOctree( const std::vector<ReachTreeNode>& roots ) {
  for ( const auto& r : roots ) {
    for ( const auto& node : hypro::preorder( r ) ) {
      assert(node.hasTimelock() != hypro::TRIBOOL::TRUE);
      mStorage.add(node.getLocation()->getName(), node.getInitialSet());
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
  spdlog::trace( "Location {}, initial box {}.", newInitialStates.begin()->first->getName(), ss.str() );
  // std::cout << "Initial set: " << Representation(newInitialStates.begin()->second.getMatrix(),
  // newInitialStates.begin()->second.getVector()).projectOn({0,1}) << std::endl;
  mAutomaton.setInitialStates( newInitialStates );
  // create roots for the reachtree from new initial states
  auto roots = hypro::makeRoots<Representation>( mAutomaton );
  // analysis
  auto reacher = ReachabilityAnalyzer( mAutomaton, settings.fixedParameters(), settings.strategy().front(), roots );
  // set up callbacks which are used by hypro to access the octree
  std::size_t                                                   shortcuts = 0;
  std::function<bool( const Representation&, const Location* )> callback  = [this, &shortcuts]( const auto& set,
                                                                                               const auto  locptr ) {
    if ( mStorage.isContained( locptr->getName(), set ) ) {
      ++shortcuts;
      return true;
    }
    return false;
  };
  hypro::ReachabilityCallbacks<Representation, Location> callbackStructure{ callback };
  reacher.setCallbacks( callbackStructure );
  // start reachability analysis
  spdlog::debug( "Start reachability analysis" );
  auto result = reacher.computeForwardReachability();
  if ( shortcuts > 0 ) {
    spdlog::debug( "Found {} fixed points by exploiting existing results.", shortcuts );
  }
  spdlog::debug( "Reachability analysis done" );

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