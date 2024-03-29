/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 22.02.22.
 */
#include <spdlog/spdlog.h>

#include <CLI/App.hpp>
#include <CLI/Config.hpp>
#include <CLI/Formatter.hpp>
#include <hypro/parser/antlr4-flowstar/ParserWrapper.h>

#include "Trainer.h"

int main( int argc, char** argv ) {
  spdlog::set_level(spdlog::level::debug);

  using namespace simplexArchitectures;
  using I   = carl::Interval<Number>;
  using IV  = std::vector<I>;
  using Box = hypro::Box<Number>;
  CLI::App app{ "Training application for simplex architectures project." };

  std::string                                     storagefilename = "trainingResult";
  std::string                                     modelfilename   = "test";
  std::map<std::string, INITIAL_STATE_HEURISTICS> map{ { "random", INITIAL_STATE_HEURISTICS::RANDOM },
                                                       { "grid", INITIAL_STATE_HEURISTICS::GRID },
                                                       { "gridCover", INITIAL_STATE_HEURISTICS::GRID_COVER } };
  TrainingSettings                                trainingSettings{ 1,
                                     INITIAL_STATE_HEURISTICS::SINGLE,
                                     { 0, 1 },
                                     Box{ IV{ I{ 0.2, 0.5 }, I{ 0.2, 0.5 }, I{ 0 }, I{ 0 }, I{ 0 } } },
                                     { 10, 10, 1, 1, 1 } };
  trainingSettings.fullCoverage = true;
  StorageSettings storageSettings{ { 0, 1, 4 }, Box{ IV{ I{ 0, 1 }, I{ 0, 1 }, I{ 0, 15 } } }, 2, 4 };
  app.add_option( "-f,--file", modelfilename, "Path to the model file" )->required()->check(CLI::ExistingFile);
  app.add_option( "-i,--iterations", trainingSettings.iterations, "Number of trainings" )->check(CLI::PositiveNumber);
  app.add_option( "-s,--strategy", trainingSettings.heuristics,
                  "Strategy/Heuristics used for training. Possible values: random, grid, gridCover" )
      ->transform( CLI::CheckedTransformer( map, CLI::ignore_case ) );
  app.add_option( "-o,--outfile", storagefilename, "Path to output file" );
  app.add_option( "-w,--width", trainingSettings.initialSetWidth, "Target width of the initial sets" )->check(CLI::PositiveNumber);

  CLI11_PARSE( app, argc, argv );

  auto [automaton,settings] = hypro::parseFlowstarFile<Number>(modelfilename);
  Storage s{storagefilename,storageSettings};
  Trainer t{ automaton, trainingSettings, s };

  s.plotCombined("pre_training");
  spdlog::info( "Start training" );
  //t.run();
  auto strat = hypro::convert(settings);

  t.run(strat, automaton.getInitialStates());
  spdlog::info( "Finished training" );
  s.plot( "post_training" );
  s.plotCombined( "post_training_combined");

  return 0;
}