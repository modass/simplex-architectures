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

#include "../simulation/Simulator.h"

#include <hypro/algorithms/reachability/Reach.h>
#include <hypro/datastructures/Hyperoctree.h>
#include <hypro/parser/antlr4-flowstar/ParserWrapper.h>
#include <hypro/util/linearOptimization/Optimizer.h>
#include <hypro/util/plotting/Plotter.h>
#include <hypro/util/plotting/HybridAutomatonPlotter.h>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

#include <CLI/App.hpp>
#include <CLI/Config.hpp>
#include <CLI/Formatter.hpp>
#include <random>
#include <string>
#include <chrono>

#include "../controller/AbstractController.h"
#include "../model_generation/generateBicycle.h"
#include "../simulation/Executor.h"
#include "../simulation/SamplingUtility.h"
#include "../simulation/Simulator.h"
#include "../training/Trainer.h"
#include "../types.h"
#include "../utility/Storage.h"
#include "../utility/StorageSettings.h"
#include "../utility/reachTreeUtility.h"
#include "../utility/treeSerialization.h"
#include "controller/BicycleBaseController.h"
#include "controller/PurePursuitController.h"
#include "utility/RaceTrack.h"
#include "ctrlConversion.h"
#include "model_generation/generateBaseController.h"
#include "utility/RaceTrack.h"

/* GENERAL ASSUMPTIONS */
// The model does *not* contain timelocks

using namespace simplexArchitectures;

using I   = carl::Interval<Number>;
using IV  = std::vector<I>;
using Box = hypro::Box<Number>;

// global constants
constexpr Eigen::Index                x     = 0;
constexpr Eigen::Index                y     = 1;
constexpr Eigen::Index                theta = 2;
constexpr Eigen::Index                tick  = 3;
constexpr Eigen::Index                v     = 4;
constexpr Eigen::Index                C     = 5;
static const std::vector<std::size_t> interesting_dimensions{ x, y, theta, tick };
// the controller influences the set of modes available to the system, so there is no direct input
static const std::vector<std::size_t> controller_dimensions{ };

int main( int argc, char* argv[] ) {
  // settings
  std::size_t               iterations{ 100 };
  std::size_t               iteration_count{ 0 };
  std::size_t               maxJumps       = 0;
  std::size_t               delta_discretization = 5; // 21
  std::pair<double, double> delta_ranges{ -60, 60 };
  std::size_t               theta_discretization = 8; // 36
  Number                    widening = 0.1;
  Number                    timeStepSize{ 0.01 };

  spdlog::set_level( spdlog::level::trace );
  // universal reference to the plotter
  auto& plt                      = hypro::Plotter<Number>::getInstance();
  plt.rSettings().xPlotInterval  = carl::Interval<double>( -2, 12 );
  plt.rSettings().yPlotInterval  = carl::Interval<double>( -2, 12 );
  plt.rSettings().dimensions = std::vector<std::size_t>{0,1};
  plt.rSettings().overwriteFiles = false;

  CLI::App app{ "Simple simulator for the car case study to check, whether the pure pursuit controller works" };
  app.add_option( "-i,--iterations", iterations, "Number of iterations/steps" )->check( CLI::PositiveNumber );
  CLI11_PARSE( app, argc, argv );
  hypro::HybridAutomaton<Number> automaton;
  hypro::ReachabilitySettings    reachSettings;
  automaton              = modelGenerator::generateBicycle( delta_ranges, delta_discretization, theta_discretization);
  reachSettings.timeStep = timeStepSize;
  std::ofstream fs{ "car.model" };
  fs << hypro::toFlowstarFormat( automaton );
  fs.close();
  {
    hypro::plotting::HybridAutomatonPlotter<Number> haplt{ automaton };
    haplt.plot();
  }

  // Hard code Racetrack
  RaceTrack track;
  track.playground = Box{ IV{ I{ 0, 10 }, I{ 0, 10 } } };
  track.obstacles  = std::vector<Box>{ Box{ IV{ I{ 3, 7 }, I{ 3, 7 } } } };
  track.waypoints  = std::vector<Point>{ Point{ 1.5, 1.5 }, Point{ 8.5, 1.5 }, Point{ 8.5, 8.5 }, Point{ 1.5, 8.5 } };

  // Hard code starting position: take first waypoint, bloat it, if wanted
  // x, y, theta, tick, v
  double               bloating        = 0.0;
  double               angularBloating = 0.0;  // 1.0472 approx. 60 degrees
  IV                   initialValuations{ I{ 1.5 - bloating,  1.5 + bloating }, I{ 1.5 - bloating, 1.5 + bloating },
                        I{ 0.1 - angularBloating, 0.1 + angularBloating }, I{ 0 }, I{ 1 } };

  auto tmpCtrl             = new PurePursuitController();
  tmpCtrl->track           = track;
  tmpCtrl->lastWaypoint    = tmpCtrl->track.waypoints.begin();
  tmpCtrl->currentWaypoint = std::next( tmpCtrl->lastWaypoint );
  spdlog::trace( "Last waypoint set to {}, current waypoint set to {}", ( *tmpCtrl->lastWaypoint ),
                 ( *tmpCtrl->currentWaypoint ) );

  auto baseCtrl             = new BicycleBaseController();
  baseCtrl->track           = track;
  baseCtrl->lastWaypoint    = baseCtrl->track.waypoints.begin();
  baseCtrl->currentWaypoint = std::next( baseCtrl->lastWaypoint );

  AbstractController<Point, Point>* advCtrl = tmpCtrl;

  // use first controller output to determine the starting location
  auto  startingpoint = hypro::conditionFromIntervals( initialValuations ).getInternalPoint();
  Point ctrlInput;
  if ( startingpoint ) {
    ctrlInput = advCtrl->generateInput( startingpoint.value() );
  } else {
    throw std::logic_error( "Initial valuations are empty, cannot compute internal point." );
  }


  // determine correct starting location in two steps: (i) chose correct theta-bucket, (ii) modify delta_bucket to match control output
  auto theta_bucket_index = getThetaBucket(startingpoint.value()[theta], theta_discretization);
  std::string theta_string = "theta_" + std::to_string(theta_bucket_index);
  LocPtr startingLocation = nullptr;
  for(auto* lptr : automaton.getLocations()) {
    if(lptr->getName().find(theta_string) != std::string::npos) {
      startingLocation = lptr;
      break;
    }
  }
  if(startingLocation == nullptr) {
    throw std::logic_error("Could not determine correct starting location");
  }

  locationConditionMap initialStates;
  initialStates.emplace(
      std::make_pair( startingLocation, hypro::conditionFromIntervals( initialValuations ) ) );
  automaton.setInitialStates(initialStates);


    auto x_min = 0.0;
    auto x_max = 7.0;
    auto y_min = 0.0;
    auto y_max = 3.0;
    auto bc_bucket_size = 0.5; // 0.5
    hypro::HybridAutomaton<Number> bcAtm = simplexArchitectures::generateBaseController(
        x_min,x_max,y_min,y_max,*baseCtrl, delta_ranges, delta_discretization, theta_discretization, bc_bucket_size, bc_bucket_size);


    IV initialValuationsBC{ I{1.5}, I{1.5}};

    auto x_bucket_index = getXBucket(startingpoint.value()[x], x_min, x_max, bc_bucket_size);
    std::string x_string = "x_" + std::to_string(x_bucket_index);
    auto y_bucket_index = getXBucket(startingpoint.value()[y], y_min, y_max, bc_bucket_size);
    std::string y_string = "y_" + std::to_string(y_bucket_index);

    LocPtr startingLocationBC = nullptr;
    for(auto* lptr : bcAtm.getLocations()) {
      if(lptr->getName().find(theta_string) != std::string::npos) {
        if(lptr->getName().find(x_string) != std::string::npos) {
          if ( lptr->getName().find( y_string ) != std::string::npos ) {
            startingLocationBC = lptr;
            break;
          }
        }
      }
    }
    if(startingLocationBC == nullptr) {
      throw std::logic_error("Could not determine correct BC starting location");
    }

    locationConditionMap initialStatesBC;
    initialStatesBC.emplace(
        std::make_pair( startingLocationBC, hypro::conditionFromIntervals( initialValuationsBC ) ) );
    bcAtm.setInitialStates(initialStatesBC);

    auto mainLocations = automaton.getLocations();
    auto variables = automaton.getVariables();
    std::map<std::string, std::vector<hypro::Location<Number>*>> variableMap;
    for (const auto& var : variables) {
      variableMap[var] = mainLocations;
    }

//    // Automata compostion:
//    std::cout << "Env #locations:" << automaton.getLocations().size() << std::endl;
//    std::cout << "BC #locations:" << bcAtm.getLocations().size() << std::endl;
//    std::cout << "Start composition" << std::endl;
//    auto start = std::chrono::steady_clock::now();
//    hypro::HybridAutomaton<Number> combinedAtm =
//        hypro::parallelCompose(automaton, bcAtm, variableMap);
//    auto end = std::chrono::steady_clock::now();
//    std::cout << "Composition finished" << std::endl;
//    std::cout << "Elapsed time in milliseconds: "
//         << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
//         << " ms" << std::endl;
//    std::cout << "Combined #locations:" << combinedAtm.getLocations().size() << std::endl;

//    return 0;

  /////////////////////////////////////////////////////////////////////
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





  // output the automaton
  spdlog::debug("Plant model:\n{}", hypro::toFlowstarFormat(automaton));

  Executor executor{ automaton, startingLocation, startingpoint.value() };
  executor.mSettings          = settings;
  executor.mExecutionSettings = ExecutionSettings{ 3, {} };


  // main loop which alternatingly invokes the controller and if necessary the analysis (training phase) for a bounded
  // number of iterations
  while ( iteration_count++ < iterations ) {
    spdlog::info( "Iteration {}", iteration_count );
    // get Controller input
    auto advControllerInput = advCtrl->generateInput( executor.mLastState );
    auto controlLocation = convertCtrlToLocation(advControllerInput, automaton, executor.mLastLocation, delta_discretization, delta_ranges);
    executor.mLastLocation = controlLocation;

    auto nextState = executor.execute( advControllerInput );

    plt.clear();
    Point adv_car = nextState.projectOn({0,1,2});
    track.addToPlotter( adv_car );
    plt.setFilename( "racetrack" );
    plt.plot2d( hypro::PLOTTYPE::png, true );
    plt.clear();
    /*
    std::stringstream ss;
    std::size_t       l = std::to_string( iterations ).size();
    ss << std::setw( l ) << std::setfill( '0' ) << iteration_count;
    storage.plotCombined( "storage_post_iteration_" + ss.str() + "_combined", false );
    auto fillsettings = hypro::Plotter<Number>::getInstance().settings();
    fillsettings.fill = true;
    if ( advControllerUsed ) {
      hypro::Plotter<Number>::getInstance().addPoint( executor.mLastState.projectOn( { 0, 1 } ),
                                                      hypro::plotting::colors[hypro::plotting::green], fillsettings );
    } else {
      hypro::Plotter<Number>::getInstance().addPoint( executor.mLastState.projectOn( { 0, 1 } ),
                                                      hypro::plotting::colors[hypro::plotting::orange], fillsettings );
    }
    hypro::Plotter<Number>::getInstance().plot2d( hypro::PLOTTYPE::png, true );
    hypro::Plotter<Number>::getInstance().clear();
     */
  }
  // the training data is automatically stored in case the trainer runs out of scope
  return 0;
}
