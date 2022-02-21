/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 20.02.22.
 */
#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>

#include "../src/training/Trainer.h"
#include "paths.h"

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

TEST_CASE( "Random Training with a single iteration" ) {
  using Interval = carl::Interval<Number>;
  using IVector  = std::vector<Interval>;
  using Box      = hypro::Box<Number>;
  // cleanup in case there is already an archive
  if ( simplexArchitectures::fileExists( "testfile" ) ) {
    std::remove( "testfile" );
  }

  simplexArchitectures::Trainer t{
      "testfile",
      simplexArchitectures::getModelsPath() + "21_simplex_watertanks_deterministic_monitor_dbg_init_ticks.model",
      { 1,
        simplexArchitectures::INITIAL_STATE_HEURISTICS::RANDOM,
        { 0, 1 },
        { Box{ IVector{ Interval{ 0, 1 }, Interval{ 0, 1 }, Interval{ 0 }, Interval{ 0 }, Interval{ 0 } } } },
        10.0,
        3 },
      { { 0, 1 }, { Box{ IVector{ Interval{ 0, 1 }, Interval{ 0, 1 } } } } } };
  t.run();
}
