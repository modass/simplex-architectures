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

#include <hypro/algorithms/reachability/Reach.h>
#include <hypro/datastructures/HybridAutomaton/HybridAutomatonComp.h>
#include <hypro/datastructures/Hyperoctree.h>
#include <hypro/parser/antlr4-flowstar/ParserWrapper.h>
#include <hypro/util/fileHandling.h>
#include <hypro/util/linearOptimization/Optimizer.h>
#include <hypro/util/plotting/Plotter.h>
#include <spdlog/spdlog.h>

#include <CLI/App.hpp>
#include <CLI/Config.hpp>
#include <CLI/Formatter.hpp>
#include <random>
#include <string>

#include "controller/AbstractController.h"
#include "controller/BicycleBaseController.h"
#include "controller/PurePursuitController.h"
#include "ctrlConversion.h"
#include "model_generation/generateBicycle.h"
#include "model_generation/generateCarModel.h"
#include "model_generation/generateSimpleBaseController.h"
#include "simulation/Executor.h"
#include "simulation/SamplingUtility.h"
#include "simulation/Simulator.h"
#include "training/Trainer.h"
#include "types.h"
#include "utility/RaceTrack.h"
#include "utility/Storage.h"
#include "utility/StorageSettings.h"
#include "utility/reachTreeUtility.h"
#include "utility/treeSerialization.h"

/* GENERAL ASSUMPTIONS */
// The model does *not* contain timelocks

using namespace simplexArchitectures;

using I            = carl::Interval<Number>;
using IV           = std::vector<I>;
using Box          = hypro::Box<Number>;
using CarAutomaton = hypro::HybridAutomatonComp<Number>;

// global constants
constexpr Eigen::Index                x     = 0;
constexpr Eigen::Index                y     = 1;
constexpr Eigen::Index                theta = 2;
constexpr Eigen::Index                tick  = 3;
constexpr Eigen::Index                v     = 4;
constexpr Eigen::Index                C     = 5;
static const std::vector<std::size_t> interesting_dimensions{ x, y, theta, tick };
// TODO
static const std::vector<std::size_t> controller_dimensions{ 2, 4 };

int main( int argc, char* argv[] ) {
  // settings
  std::size_t               iterations{ 0 };
  std::size_t               iteration_count{ 0 };
  std::size_t               maxJumps             = 200;
  std::size_t               theta_discretization = 36;
  std::pair<double, double> delta_ranges{ -60, 60 };
  Number                    widening = 0.1;
  bool                      training = true;
  std::string               storagefilename{ "storage_car" };
  std::string               composedAutomatonFile{ "composedAutomaton.model" };
  Number                    timeStepSize{ 0.01 };

  spdlog::set_level( spdlog::level::trace );
  // universal reference to the plotter
  auto& plt                      = hypro::Plotter<Number>::getInstance();
  plt.rSettings().overwriteFiles = false;

  CLI::App app{ "Training application for simplex architectures project." };
  app.add_option( "-s,--storage", storagefilename, "Path to file with stored sets" );
  app.add_option( "-i,--iterations", iterations, "Number of iterations/steps" )->check( CLI::PositiveNumber );
  app.add_option( "-c,--composedAutomaton", composedAutomatonFile, "Path to the file storing the composedAutomaton" );
  app.add_flag( "--learn", training,
                "If given, the method will try to add new initial sets to the safe area, if they are safe. Otherwise "
                "the analysis terminates?" );
  CLI11_PARSE( app, argc, argv );
  // parse model
  hypro::ReachabilitySettings reachSettings;
  auto                        carModel = modelGenerator::generateCarModel( theta_discretization );
  reachSettings.timeStep               = timeStepSize;
  {
    std::ofstream fs{ "car.model" };
    fs << hypro::toFlowstarFormat( carModel );
    fs.close();
  }

  // Hard code Racetrack
  RaceTrack track;
  track.playground = Box{ IV{ I{ 0, 10 }, I{ 0, 10 } } };
  track.obstacles  = std::vector<Box>{ Box{ IV{ I{ 3, 7 }, I{ 3, 7 } } } };
  track.waypoints  = std::vector<Point>{ Point{ 1.5, 1.5 }, Point{ 8.5, 1.5 }, Point{ 8.5, 8.5 }, Point{ 1.5, 8.5 } };

  // Hard code starting position: take first waypoint, bloat it, if wanted
  // x, y, theta, tick, v
  double bloating        = 0.0;
  double angularBloating = 0.0;  // 1.0472 approx. 60 degrees
  IV     initialValuations{ I{ 1.5 - bloating, 1.5 + bloating }, I{ 1.5 - bloating, 1.5 + bloating }, I{ 0 }, I{ 0 },
                        I{ 1 } };

  auto tmpCtrl             = new PurePursuitController();
  tmpCtrl->track           = track;
  tmpCtrl->lastWaypoint    = tmpCtrl->track.waypoints.begin();
  tmpCtrl->currentWaypoint = std::next( tmpCtrl->lastWaypoint );

  AbstractController<Point, Point>* advCtrl = tmpCtrl;

  // use first controller output to determine the starting location
  auto  startingpoint = hypro::conditionFromIntervals( initialValuations ).getInternalPoint();
  Point ctrlInput;
  if ( startingpoint ) {
    ctrlInput = advCtrl->generateInput( startingpoint.value() );
  } else {
    throw std::logic_error( "Initial valuations are empty, cannot compute internal point." );
  }

  // determine correct starting location in two steps: (i) chose correct theta-bucket, (ii) modify delta_bucket to match
  // control output
  auto        theta_bucket_index = 0;
  std::string theta_string       = "theta_" + std::to_string( theta_bucket_index );
  LocPtr      startingLocation   = nullptr;
  for ( auto* lptr : carModel.getLocations() ) {
    if ( lptr->getName().find( theta_string ) != std::string::npos ) {
      startingLocation = lptr;
      break;
    }
  }
  if ( startingLocation == nullptr ) {
    throw std::logic_error( "Could not determine correct starting location" );
  }

  locationConditionMap initialStates;
  initialStates.emplace( std::make_pair( startingLocation, hypro::conditionFromIntervals( initialValuations ) ) );
  carModel.setInitialStates( initialStates );

  RoadSegment segment = { 0.0, 0.0, 7.0, 3.0, LeftToRight };

  auto bcAtm = simplexArchitectures::generateSimpleBaseController( theta_discretization, 0.5, 0.5, M_PI / 12.0,
                                                                   M_PI / 4.5, { segment } );

  IV initialValuationsBC{ I{ 1.5 }, I{ 1.5 } };

  LocPtr startingLocationBC = nullptr;
  for ( auto* lptr : bcAtm.getLocations() ) {
    if ( lptr->getName().find( "segment_0_zone_2" ) != std::string::npos ) {
      startingLocationBC = lptr;
      break;
    }
  }
  if ( startingLocationBC == nullptr ) {
    throw std::logic_error( "Could not determine correct BC starting location" );
  }

  locationConditionMap initialStatesBC;
  initialStatesBC.emplace( std::make_pair( startingLocationBC, hypro::conditionFromIntervals( initialValuationsBC ) ) );
  bcAtm.setInitialStates( initialStatesBC );

  // Automata compostion:
  auto                                                         mainLocations = carModel.getLocations();
  auto                                                         variables     = carModel.getVariables();
  std::map<std::string, std::vector<hypro::Location<Number>*>> variableMap;
  for ( const auto& var : variables ) {
    variableMap[var] = mainLocations;
  }

  hypro::HybridAutomaton<Number> automaton;
  if ( hypro::file_exists( composedAutomatonFile ) ) {
    std::cout << "Load composed automaton from file." << std::endl;
    hypro::ReachabilitySettings s;
    std::tie( automaton, s ) = hypro::parseFlowstarFile<Number>( composedAutomatonFile );
  } else {
    // Automata compostion:
    std::cout << "Env #locations:" << carModel.getLocations().size() << std::endl;
    std::cout << "BC #locations:" << bcAtm.getLocations().size() << std::endl;
    std::cout << "Start composition" << std::endl;
    auto start = std::chrono::steady_clock::now();
    automaton  = hypro::parallelCompose( carModel, bcAtm, variableMap );
    auto end   = std::chrono::steady_clock::now();
    std::cout << "Composition finished" << std::endl;
    std::cout << "Elapsed time in milliseconds: "
              << std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count() << " ms" << std::endl;
    std::cout << "Combined #locations:" << automaton.getLocations().size() << std::endl;
    {
      std::ofstream fs{ composedAutomatonFile };
      fs << hypro::toFlowstarFormat( automaton );
      fs.close();
    }
  }

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

  // the executor runs on the car model only
  assert( carModel.getInitialStates().size() == 1 );
  Executor executor{ carModel, carModel.getInitialStates().begin()->first, startingpoint.value() };
  executor.mSettings          = settings;
  executor.mExecutionSettings = ExecutionSettings{ 3, { theta, v } };
  executor.mPlot              = false;

  // Storage for trained sets
  auto storagesettings = StorageSettings{
      interesting_dimensions,
      Box{ IV{ I{ segment.x_min, segment.x_max }, I{ segment.y_min, segment.y_max }, I{ 0, 360 }, I{ 0, 1 } } } };
  auto             storage = Storage( storagefilename, storagesettings );
  TrainingSettings trainingSettings{
      1,                                                                  // iterations
      INITIAL_STATE_HEURISTICS::SINGLE,                                   // heuristics
      { 0, 1 },                                                           // widening dimensions
      Box{ IV{ I{ 0.2, 0.5 }, I{ 0.2, 0.5 }, I{ 0 }, I{ 0 }, I{ 0 } } },  // sampling area
      { 10, 10, 1, 1, 1 },                                                // subdivisions per dimension
      carl::convert<hypro::tNumber, Number>( settings.fixedParameters().localTimeHorizon ),  // time horizon
      maxJumps,                                                                              // jump depth
      widening,                                                                              // initial set width
      false };                                                                               // try full coverage?
  // teh trainer runs on the combined automaton of base controller and car model
  Trainer trainer{ automaton, trainingSettings, storage };

  // monitor/simulator, runs on the car model
  Simulator sim{ carModel, settings, storage };
  sim.mLastStates.emplace( std::make_pair( executor.mLastLocation, std::set<Point>{ executor.mLastState } ) );

  if ( training && storage.size() == 0 ) {
    auto initialStates = std::map<LocPtr, hypro::Condition<Number>>{};
    initialStates.emplace( std::make_pair(
        executor.mLastLocation, hypro::Condition<Number>( widenSample( startingpoint.value(), widening,
                                                                       trainingSettings.wideningDimensions ) ) ) );
    trainer.run( settings, initialStates );
    storage.plotCombined( "storage_post_initial_training_combined" );
  }

  // main loop which alternatingly invokes the controller and if necessary the analysis (training phase) for a bounded
  // number of iterations
  while ( iteration_count++ < iterations ) {
    spdlog::info( "Iteration {}", iteration_count );
    // get AC Controller input
    Point advControllerInput = advCtrl->generateInput( executor.mLastState );

    // 1 simulation advanced controller, starting from initialvalue & location
    {
      std::stringstream s;
      s << advControllerInput;
      spdlog::info( "Start advanced controller simulation with controller output {}", s.str() );
    }
    hypro::TRIBOOL advControllerSafe = sim.isSafe( advControllerInput );
    bool           advControllerUsed = true;

    // if all safe & last point in reach set, pointify resulting set, update initialstate, update monitor (current
    // point)
    if ( advControllerSafe == hypro::TRIBOOL::TRUE ) {
      {
        std::stringstream ss;
        ss << advControllerInput;
        spdlog::debug( "Advanced controller is safe and traces end in known safe area, run with output {}", ss.str() );
      }
      executor.execute( advControllerInput );
      // TODO why does the simulator require the last used control input?
      sim.update( advControllerInput, executor.mLastState );
    } else if ( advControllerSafe == hypro::TRIBOOL::FALSE ) {
      {
        std::stringstream ss;
        ss << sim.getBaseControllerOutput();
        spdlog::debug( "Advanced controller is unsafe, use base controller with output {}", ss.str() );
      }
      // TODO why do we call the simulator to get the BC output, should we not call the BC directly?
      executor.execute( sim.getBaseControllerOutput() );
      sim.update( sim.getBaseControllerOutput(), executor.mLastState );
      advControllerUsed = false;
    } else {
      spdlog::debug( "Advanced controller is safe (bounded time), but traces end in unknown safety area" );
      bool allSafe = false;
      /*if ( training ) {
        spdlog::info( "Start training for {} locations", sim.unknownSamples.size() );
        allSafe = true;
        for ( const auto& [loc, setVector] : sim.unknownSamples ) {
          for ( const auto& set : setVector ) {
            locationConditionMap initialConfigurations{};
            auto                 setIntervals = set.intervals();
            setIntervals[0].bloat_by( 0.02 );
            setIntervals[1].bloat_by( 0.02 );
            initialConfigurations[loc] = hypro::Condition( setIntervals );
            auto              safe     = trainer.run( settings, initialConfigurations );
            std::stringstream ss;
            ss << set;
            spdlog::debug( "Training result for location {} and set {}: {}", loc->getName(), ss.str(), safe );
            allSafe = allSafe && safe;
          }
        }
        std::stringstream ss;
        std::size_t       l = std::to_string( iterations ).size();
        ss << std::setw( l ) << std::setfill( '0' ) << iteration_count;
        storage.plotCombined( "storage_post_training_" + ss.str() + "_combined" );
      }*/
      if ( !allSafe ) {
        {
          std::stringstream ss;
          ss << sim.getBaseControllerOutput();
          spdlog::debug( "Not all sets were safe (unbounded time), run base controller with output {}", ss.str() );
        }
        executor.execute( sim.getBaseControllerOutput() );
        sim.update( sim.getBaseControllerOutput(), executor.mLastState );
        advControllerUsed = false;
      } else {
        {
          std::stringstream ss;
          ss << advControllerInput;
          spdlog::debug( "All sets were safe (unbounded time), run advanced controller with output {}", ss.str() );
        }
        executor.execute( advControllerInput );
        sim.update( advControllerInput, executor.mLastState );
      }
    }

    /*
    std::stringstream ss;
    std::size_t       l = std::to_string( iterations ).size();
    ss << std::setw( l ) << std::setfill( '0' ) << iteration_count;
    storage.plotCombined( "storage_post_iteration_" + ss.str() + "_combined", false );
     */
    {
      auto fillsettings = hypro::Plotter<Number>::getInstance().settings();
      fillsettings.fill = true;
      if ( advControllerUsed ) {
        hypro::Plotter<Number>::getInstance().addPoint( executor.mLastState.projectOn( { 0, 1 } ),
                                                        hypro::plotting::colors[hypro::plotting::green], fillsettings );
      } else {
        hypro::Plotter<Number>::getInstance().addPoint(
            executor.mLastState.projectOn( { 0, 1 } ), hypro::plotting::colors[hypro::plotting::orange], fillsettings );
      }
      hypro::Plotter<Number>::getInstance().plot2d( hypro::PLOTTYPE::png, true );
      hypro::Plotter<Number>::getInstance().clear();
    }
  }
  // the training data is automatically stored in case the trainer runs out of scope
  return 0;
}
