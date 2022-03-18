/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 16.02.22.
 */

/*
 * Copyright (c) 2022.
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 28.10.21.
 */

#include <hypro/algorithms/reachability/Reach.h>
#include <hypro/datastructures/Hyperoctree.h>
#include <hypro/parser/antlr4-flowstar/ParserWrapper.h>
#include <hypro/util/linearOptimization/Optimizer.h>
#include <hypro/util/plotting/Plotter.h>
#include <spdlog/spdlog.h>

#include <CLI/App.hpp>
#include <CLI/Config.hpp>
#include <CLI/Formatter.hpp>
#include <random>
#include <string>

#include "../controller/AbstractController.h"
#include "../controller/ConstantController.h"
#include "../controller/RLController.h"
#include "../simulation/Executor.h"
#include "../simulation/SamplingUtility.h"
#include "../simulation/Simulator.h"
#include "../training/Trainer.h"
#include "../types.h"
#include "../utility/Storage.h"
#include "../utility/StorageSettings.h"
#include "../utility/reachTreeUtility.h"
#include "../utility/treeSerialization.h"

/* GENERAL ASSUMPTIONS */
// The model does *not* contain timelocks

using namespace simplexArchitectures;

using I   = carl::Interval<Number>;
using IV  = std::vector<I>;
using Box = hypro::Box<Number>;

// global constants
static const std::vector<std::size_t> interesting_dimensions{ 0, 1, 3 };
static const std::vector<std::size_t> controller_dimensions{ 2 };

int main( int argc, char* argv[] ) {
  // settings
  std::size_t                                       iterations{ 5 };
  std::size_t                                       iteration_count{ 0 };
  std::size_t                                       maxJumps = 200;
  Number                                            widening = 0.1;
  bool                                              training = true;
  std::string                                       modelfilename;
  std::string                                       storagefilename;

  spdlog::set_level(spdlog::level::trace);

  CLI::App app{ "Training application for simplex architectures project." };
  app.add_option( "-m,--model", modelfilename, "Path to the model file" )->required()->check( CLI::ExistingFile );
  app.add_option( "-s,--storage", storagefilename, "Path to file with stored sets" )->required();
  app.add_option( "-i,--iterations", iterations, "Number of iterations/steps" )->check( CLI::PositiveNumber );
  app.add_flag( "--learn", training,
                "If given, the method will try to add new initial sets to the safe area, if they are safe. Otherwise "
                "the analysis terminates?" );
  CLI11_PARSE( app, argc, argv );
  // parse model
  auto [automaton, reachSettings] = hypro::parseFlowstarFile<Number>( modelfilename );
  // reachability analysis settings, here only used for simulation
  auto settings                                                   = hypro::convert( reachSettings );
  settings.rStrategy().front().detectJumpFixedPoints              = true;
  settings.rStrategy().front().detectFixedPointsByCoverage        = true;
  settings.rStrategy().front().detectContinuousFixedPointsLocally = true;
  settings.rStrategy().front().detectZenoBehavior                 = true;
  settings.rStrategy().front().numberSetsForContinuousCoverage    = 2;
  settings.rFixedParameters().localTimeHorizon                    = 200;
  settings.rFixedParameters().jumpDepth                           = maxJumps;
  settings.rStrategy().begin()->aggregation                       = hypro::AGG_SETTING::AGG;
  // random controller
  //AbstractController<Point, Point>* advCtrl = new ConstantController<Point, Point>( Point{ 0 } );
  if(!fileExists("/home/stefan/tu/repositories/simplex-architectures/networks/watertanks.txt")) {
    throw std::ios_base::failure("File does not exist.");
  }
  AbstractController<Point, Point>* advCtrl = new RLController("/home/stefan/tu/repositories/simplex-architectures/networks/watertanks.txt");
  // initialize Executor
  std::optional<Point> initialValuation = automaton.getInitialStates().begin()->second.getInternalPoint();
  if ( !initialValuation ) {
    throw std::logic_error( "Initial set is empty, abort." );
  }
  Executor executor{ automaton, automaton.getInitialStates().begin()->first, initialValuation.value() };
  executor.mSettings = settings;
  // initial trainging, if required, otherwise just load the treefile and update the local variable (trees)
  // Storagesettings will be overidden if a file with data exists
  StorageSettings  storageSettings{ { 0, 1, 4 }, Box{ IV{ I{ 0, 1 }, I{ 0, 1 }, I{ 0, 15 } } }, 2, 4 };
  TrainingSettings trainingSettings{
      1,
      INITIAL_STATE_HEURISTICS::SINGLE,
      { 0, 1 },
      Box{ IV{ I{ 0.2, 0.5 }, I{ 0.2, 0.5 }, I{ 0 }, I{ 0 }, I{ 0 } } },
      { 10, 10, 1, 1, 1 },
      carl::convert<hypro::tNumber, Number>( settings.fixedParameters().localTimeHorizon ),
      maxJumps,
      widening,
      false };
  Storage storage{ storagefilename, storageSettings };
  storage.plotCombined("storage_post_loading_combined");
  Trainer trainer{ automaton, trainingSettings, storage };
  // monitor
  Simulator sim{ automaton, settings, storage };
  sim.mLastStates.emplace( std::make_pair( executor.mLastLocation, std::set<Point>{ executor.mLastState } ) );

  if ( training && storage.size() == 0 ) {
    auto                                initialStates = std::map<LocPtr, hypro::Condition<Number>>{};
    initialStates.emplace( std::make_pair( executor.mLastLocation, hypro::Condition<Number>( widenSample(initialValuation.value(),widening,{0,1}) ) ) );
    trainer.run( settings, initialStates );
    storage.plotCombined("storage_post_initial_training_combined");
  }

  // main loop which alternatingly invokes the controller and if necessary the analysis (training phase) for a bounded
  // number of iterations
  while ( iteration_count++ < iterations ) {
    spdlog::info( "Iteration {}", iteration_count );
    // get Controller input
    Point advControllerInput = advCtrl->generateInput( executor.mLastState );

    // 1 simulation advanced controller, starting from initialvalue & location
    std::stringstream s;
    s << advControllerInput;
    spdlog::info( "Start advanced controller simulation with controller output {}", s.str() );
    hypro::TRIBOOL advControllerSafe = sim.isSafe( advControllerInput );
    bool advControllerUsed = true;

    // if all safe & last point in reach set, pointify resulting set, update initialstate, update monitor (current
    // point)
    if ( advControllerSafe == hypro::TRIBOOL::TRUE ) {
      std::stringstream ss;
      ss << advControllerInput;
      spdlog::debug("Advanced controller is safe and traces end in known safe area, run with output {}", ss.str());
      executor.execute( advControllerInput );
      sim.update(advControllerInput,executor.mLastState);
    } else if ( advControllerSafe == hypro::TRIBOOL::FALSE ) {
      std::stringstream ss;
      ss << sim.getBaseControllerOutput();
      spdlog::debug("Advanced controller is unsafe, use base controller with output {}", ss.str());
      executor.execute( sim.getBaseControllerOutput() );
      sim.update(sim.getBaseControllerOutput(),executor.mLastState);
      advControllerUsed = false;
    } else {
      spdlog::debug("Advanced controller is safe (bounded time), but traces end in unknown safety area");
      bool allSafe = false;
      if ( training ) {
        spdlog::info("Start training for {} locations", sim.unknownSamples.size());
        allSafe = true;
        for ( const auto& [loc, setVector] : sim.unknownSamples ) {
          for ( const auto& set : setVector ) {
            locationConditionMap initialConfigurations{};
            auto setIntervals = set.intervals();
            setIntervals[0].bloat_by(0.02);
            setIntervals[1].bloat_by(0.02);
            initialConfigurations[loc] = hypro::Condition(setIntervals);
            auto safe                  = trainer.run( settings, initialConfigurations );
            std::stringstream ss;
            ss << set;
            spdlog::debug("Training result for location {} and set {}: {}", loc->getName(), ss.str() , safe);
            allSafe                    = allSafe && safe;
          }
        }
        std::stringstream ss;
        std::size_t l = std::to_string(iterations).size();
        ss << std::setw(l) << std::setfill('0') << iteration_count;
        storage.plotCombined("storage_post_training_" + ss.str() + "_combined");
      }
      if ( !allSafe ) {
        std::stringstream ss;
        ss <<sim.getBaseControllerOutput();
        spdlog::debug("Not all sets were safe (unbounded time), run base controller with output {}", ss.str() );
        executor.execute( sim.getBaseControllerOutput() );
        sim.update(sim.getBaseControllerOutput(),executor.mLastState);
        advControllerUsed = false;
      } else {
        std::stringstream ss;
        ss <<advControllerInput;
        spdlog::debug("All sets were safe (unbounded time), run advanced controller with output {}", ss.str());
        executor.execute( advControllerInput );
        sim.update(advControllerInput,executor.mLastState);
      }
    }

    std::stringstream ss;
    std::size_t l = std::to_string(iterations).size();
    ss << std::setw(l) << std::setfill('0') << iteration_count;
    storage.plotCombined("storage_post_iteration_" + ss.str() + "_combined", false);
    auto fillsettings = hypro::Plotter<Number>::getInstance().settings();
    fillsettings.fill = true;
    if(advControllerUsed) {
      hypro::Plotter<Number>::getInstance().addPoint(executor.mLastState.projectOn({0,1}), hypro::plotting::colors[hypro::plotting::green], fillsettings);
    } else {
      hypro::Plotter<Number>::getInstance().addPoint(executor.mLastState.projectOn({0,1}), hypro::plotting::colors[hypro::plotting::orange], fillsettings);
    }
    hypro::Plotter<Number>::getInstance().plot2d(hypro::PLOTTYPE::png, true);
    hypro::Plotter<Number>::getInstance().clear();
  }
  // the training data is automatically stored in case the trainer runs out of scope
  return 0;
}
