/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 20.02.22.
 */
#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <hypro/parser/antlr4-flowstar/ParserWrapper.h>

#include "../src/training/Trainer.h"
#include "paths.h"
#include "../src/training/TrainingHeuristics.h"

TEST_CASE( "Construct Trainer" ) {
  // cleanup in case there is already an archive
  if ( simplexArchitectures::fileExists( "testfile" ) ) {
    std::remove( "testfile" );
  }
  {
    simplexArchitectures::Trainer t{
        "testfile", "modelfile", { 1, simplexArchitectures::INITIAL_STATE_HEURISTICS::RANDOM, {}, {} }, { {}, {} } };
  }
  // create second trainer, this time it should read the file which was created when the previous trainer ran out of
  // scope
  {
    simplexArchitectures::Trainer t{
        "testfile", "modelfile", { 1, simplexArchitectures::INITIAL_STATE_HEURISTICS::RANDOM, {}, {} }, { {}, {} } };
  }
}

TEST_CASE( "Grid-sample generation", "[training]" ) {
  using Interval = carl::Interval<Number>;
  using IVector  = std::vector<Interval>;
  using Box      = hypro::Box<Number>;
  auto [automaton, _] =
      hypro::parseFlowstarFile<Number>( simplexArchitectures::getModelsPath() + "21_simplex_watertanks.model" );
  std::vector<std::size_t>               resolution{ 2, 2, 1, 1, 1 };
  simplexArchitectures::TrainingSettings trainingSettings{
      5,
      simplexArchitectures::INITIAL_STATE_HEURISTICS::GRID,
      { 0, 1 },
      Box{ IVector{ Interval{ 0, 1 }, Interval{ 0, 1 }, Interval{ 0 }, Interval{ 0 }, Interval{ 0 } } } };
  simplexArchitectures::Grid g{ resolution, trainingSettings };
  auto                       s1 = g( automaton, trainingSettings );
  CAPTURE( std::begin( s1 )->second );
  std::cout << std::begin( s1 )->second << std::endl;
  REQUIRE( s1.size() == 1 );
  REQUIRE( std::begin( s1 )->second.contains( hypro::Point<Number>{ 0.33, 0.33, 0, 0, 0 } ) );
  REQUIRE( !std::begin( s1 )->second.contains( hypro::Point<Number>{ 0.66, 0.66, 0, 0, 0 } ) );
  REQUIRE( !std::begin( s1 )->second.contains( hypro::Point<Number>{ 0.25, 0.25, 1, 0, 0 } ) );
  auto b1 = Box( std::begin( s1 )->second.getMatrix(), std::begin( s1 )->second.getVector() );
  REQUIRE( b1.intervals()[0].contains( Interval{ 0.29, 0.38 } ) );
  REQUIRE( b1.intervals()[1].contains( Interval{ 0.29, 0.38 } ) );
  REQUIRE( b1.intervals()[2] == Interval{ 0.0 } );
  REQUIRE( b1.intervals()[3] == Interval{ 0.0 } );
  REQUIRE( b1.intervals()[4] == Interval{ 0.0 } );

  auto s2 = g( automaton, trainingSettings );
  REQUIRE( s2.size() == 1 );
  REQUIRE( std::begin( s2 )->second.contains( hypro::Point<Number>{ 0.66, 0.33, 0, 0, 0 } ) );
  REQUIRE( !std::begin( s2 )->second.contains( hypro::Point<Number>{ 0.66, 0.66, 0, 0, 0 } ) );
  REQUIRE( !std::begin( s2 )->second.contains( hypro::Point<Number>{ 0.25, 0.25, 1, 0, 0 } ) );
  auto s3 = g( automaton, trainingSettings );
  REQUIRE( s3.size() == 1 );
  REQUIRE( std::begin( s3 )->second.contains( hypro::Point<Number>{ 0.33, 0.66, 0, 0, 0 } ) );
  REQUIRE( !std::begin( s3 )->second.contains( hypro::Point<Number>{ 0.66, 0.66, 0, 0, 0 } ) );
  REQUIRE( !std::begin( s3 )->second.contains( hypro::Point<Number>{ 0.25, 0.25, 1, 0, 0 } ) );
  auto s4 = g( automaton, trainingSettings );
  REQUIRE( s4.size() == 1 );
  REQUIRE( std::begin( s4 )->second.contains( hypro::Point<Number>{ 0.66, 0.66, 0, 0, 0 } ) );
  REQUIRE( !std::begin( s4 )->second.contains( hypro::Point<Number>{ 0.33, 0.66, 0, 0, 0 } ) );
  REQUIRE( !std::begin( s4 )->second.contains( hypro::Point<Number>{ 0.25, 0.25, 1, 0, 0 } ) );
}

TEST_CASE( "Grid-cover-sample generation", "[training]" ) {
  using Interval = carl::Interval<Number>;
  using IVector  = std::vector<Interval>;
  using Box      = hypro::Box<Number>;
  auto [automaton, _] =
      hypro::parseFlowstarFile<Number>( simplexArchitectures::getModelsPath() + "21_simplex_watertanks.model" );
  std::vector<std::size_t>               resolution{ 2, 2, 1, 1, 1 };
  simplexArchitectures::TrainingSettings trainingSettings{
      5,
      simplexArchitectures::INITIAL_STATE_HEURISTICS::GRID_COVER,
      { 0, 1 },
      Box{ IVector{ Interval{ 0, 1 }, Interval{ 0, 1 }, Interval{ 0 }, Interval{ 0 }, Interval{ 0 } } } };
  simplexArchitectures::GridCover g{ resolution, trainingSettings };
  auto                            s1 = g( automaton, trainingSettings );
  CAPTURE( std::begin( s1 )->second );
  std::cout << std::begin( s1 )->second << std::endl;
  REQUIRE( s1.size() == 1 );
  REQUIRE( std::begin( s1 )->second.dimension() == trainingSettings.samplingArea.dimension() );
  REQUIRE( std::begin( s1 )->second.contains( hypro::Point<Number>{ 0.33, 0.33, 0, 0, 0 } ) );
  REQUIRE( !std::begin( s1 )->second.contains( hypro::Point<Number>{ 0.66, 0.66, 0, 0, 0 } ) );
  REQUIRE( !std::begin( s1 )->second.contains( hypro::Point<Number>{ 0.25, 0.25, 1, 0, 0 } ) );
  auto b1 = Box( std::begin( s1 )->second.getMatrix(), std::begin( s1 )->second.getVector() );
  REQUIRE( b1.intervals()[0] == Interval{ 0.0, 0.5 } );
  REQUIRE( b1.intervals()[1] == Interval{ 0.0, 0.5 } );
  REQUIRE( b1.intervals()[2] == Interval{ 0.0 } );
  REQUIRE( b1.intervals()[3] == Interval{ 0.0 } );
  REQUIRE( b1.intervals()[4] == Interval{ 0.0 } );

  auto s2 = g( automaton, trainingSettings );
  REQUIRE( s2.size() == 1 );
  REQUIRE( std::begin( s2 )->second.contains( hypro::Point<Number>{ 0.66, 0.33, 0, 0, 0 } ) );
  REQUIRE( !std::begin( s2 )->second.contains( hypro::Point<Number>{ 0.66, 0.66, 0, 0, 0 } ) );
  REQUIRE( !std::begin( s2 )->second.contains( hypro::Point<Number>{ 0.25, 0.25, 1, 0, 0 } ) );
  auto b2 = Box( std::begin( s2 )->second.getMatrix(), std::begin( s2 )->second.getVector() );
  REQUIRE( b2.intervals()[0] == Interval{ 0.5, 1.0 } );
  REQUIRE( b2.intervals()[1] == Interval{ 0.0, 0.5 } );
  REQUIRE( b2.intervals()[2] == Interval{ 0.0 } );
  REQUIRE( b2.intervals()[3] == Interval{ 0.0 } );
  REQUIRE( b2.intervals()[4] == Interval{ 0.0 } );
  auto s3 = g( automaton, trainingSettings );
  REQUIRE( s3.size() == 1 );
  REQUIRE( std::begin( s3 )->second.contains( hypro::Point<Number>{ 0.33, 0.66, 0, 0, 0 } ) );
  REQUIRE( !std::begin( s3 )->second.contains( hypro::Point<Number>{ 0.66, 0.66, 0, 0, 0 } ) );
  REQUIRE( !std::begin( s3 )->second.contains( hypro::Point<Number>{ 0.25, 0.25, 1, 0, 0 } ) );
  auto b3 = Box( std::begin( s3 )->second.getMatrix(), std::begin( s3 )->second.getVector() );
  REQUIRE( b3.intervals()[0] == Interval{ 0.0, 0.5 } );
  REQUIRE( b3.intervals()[1] == Interval{ 0.5, 1.0 } );
  REQUIRE( b3.intervals()[2] == Interval{ 0.0 } );
  REQUIRE( b3.intervals()[3] == Interval{ 0.0 } );
  REQUIRE( b3.intervals()[4] == Interval{ 0.0 } );
  auto s4 = g( automaton, trainingSettings );
  REQUIRE( s4.size() == 1 );
  REQUIRE( std::begin( s4 )->second.contains( hypro::Point<Number>{ 0.66, 0.66, 0, 0, 0 } ) );
  REQUIRE( !std::begin( s4 )->second.contains( hypro::Point<Number>{ 0.33, 0.66, 0, 0, 0 } ) );
  REQUIRE( !std::begin( s4 )->second.contains( hypro::Point<Number>{ 0.25, 0.25, 1, 0, 0 } ) );
  auto b4 = Box( std::begin( s4 )->second.getMatrix(), std::begin( s4 )->second.getVector() );
  REQUIRE( b4.intervals()[0] == Interval{ 0.5, 1.0 } );
  REQUIRE( b4.intervals()[1] == Interval{ 0.5, 1.0 } );
  REQUIRE( b4.intervals()[2] == Interval{ 0.0 } );
  REQUIRE( b4.intervals()[3] == Interval{ 0.0 } );
  REQUIRE( b4.intervals()[4] == Interval{ 0.0 } );
}
