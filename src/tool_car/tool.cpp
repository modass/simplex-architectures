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
#include <regex>
#include <string>

#include "controller/AbstractController.h"
#include "controller/BicycleBaseController.h"
#include "controller/PurePursuitController.h"
#include "controller/RandomCarController.h"
#include "ctrlConversion.h"
#include "model_generation/generateCarModel.h"
#include "model_generation/generateCarBaseController.h"
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
#include "../../racetracks/austria/bad_states.h"
#include "../../racetracks/austria/segments.h"
#include "../../racetracks/austria/waypoints.h"
#include "../../racetracks/austria/playground.h"
#include "../../racetracks/austria/optimized_waypoints.h"

/* GENERAL ASSUMPTIONS */
// The model does *not* contain timelocks

using namespace simplexArchitectures;

using I            = carl::Interval<Number>;
using IV           = std::vector<I>;
using Box          = hypro::Box<Number>;

// global constants
constexpr Eigen::Index                x     = 0;
constexpr Eigen::Index                y     = 1;
constexpr Eigen::Index                theta = 2;
constexpr Eigen::Index                tick  = 3;
constexpr Eigen::Index                v     = 4;
constexpr Eigen::Index                C     = 5;
static const std::vector<std::size_t> interesting_dimensions{ x, y };
// TODO
static const std::vector<std::size_t> controller_dimensions{ theta, v };

Point executeWithLapCount( Executor<Automaton>& executor, const Point& advControllerInput, size_t& lapCounter,
                           const RaceTrack& racetrack ) {
  auto old_pos_x = executor.mLastState[0];
  auto old_pos_y = executor.mLastState[1];
  auto new_pos   = executor.execute( advControllerInput );
  auto new_pos_x = new_pos[0];
  if ( racetrack.startFinishYlow <= old_pos_y && old_pos_y <= racetrack.startFinishYhigh &&
       old_pos_x < racetrack.startFinishX && racetrack.startFinishX <= new_pos_x ) {
    lapCounter++;
    spdlog::info( "Lap {}", lapCounter );
  }
  return new_pos;
}

int main( int argc, char* argv[] ) {
  // settings
  std::size_t               iterations{ 0 };
  std::size_t               iteration_count{ 0 };
  std::size_t               maxJumps             = 200;
  std::size_t               theta_discretization = 36;
  std::pair<double, double> delta_ranges{ -60, 60 };
  Number                    widening = 0.2;
  bool                      training = true;
  bool                      alwaysUseAC = false; // use the ac even if it is unsafe
  bool                      alwaysUseBC = false; // use the bc even if the AC is safe
  std::string               storagefilename{ "storage_car" };
  std::string               composedAutomatonFile{ "composedAutomaton.model" };
  Number                    timeStepSize{ 0.01 };
  Number                    cycleTime{ 0.1 };
  std::size_t               trackID{ 0 };
  bool                      plotSets      = false;
  bool                      plotPosition  = false;
  bool                      plotRaceTrack = true;

  spdlog::set_level( spdlog::level::info );
  // universal reference to the plotter
  auto& plt                      = hypro::Plotter<Number>::getInstance();
  plt.rSettings().overwriteFiles = false;

  CLI::App app{ "Training application for simplex architectures project." };
  app.add_option( "-s,--storage", storagefilename, "Path to file with stored sets" );
  app.add_option( "-i,--iterations", iterations, "Number of iterations/steps" )->check( CLI::PositiveNumber );
  app.add_option( "-c,--composedAutomaton", composedAutomatonFile, "Path to the file storing the composedAutomaton" );
  app.add_flag( "--training", training,
                "If given, the method will try to add new initial sets to the safe area, if they are safe. Otherwise "
                "the analysis terminates?" );
  CLI11_PARSE( app, argc, argv );

  // Hard code Racetracks


  RaceTrack                track;

  switch(trackID) {
    case 0:
      track.playground = createPlayground<Number>();
      track.obstacles = createBadStates<hypro::HybridAutomaton<Number>>();
      track.roadSegments = createSegments<GeneralRoadSegment>();
      //track.waypoints = createWaypoints<Number>();
      track.waypoints = createOptimizedWaypoints<Number>();
      track.startFinishX = 5.0;
      track.startFinishYlow = 0.0;
      track.startFinishYhigh = 3.0;
//    case 0: // square-shaped track
//      track.playground = Box{ IV{ I{ 0, 10 }, I{ 0, 10 } } };
//      track.obstacles  = std::vector<Box>{ Box{ IV{ I{ 3, 7 }, I{ 3, 7 } } } };
//      track.waypoints  = std::vector<Point>{ Point{ 1.5, 1.5 }, Point{ 8.5, 1.5 }, Point{ 8.5, 8.5 }, Point{ 1.5, 8.5 } };
//
//      track.roadSegments = { {Point{3, 3}, Point{0, 0}, Point{7, 3}, Point{10,0} },
//                             {Point{7, 3}, Point{10,0}, Point{7, 7}, Point{10,10} },
//                             {Point{7, 7}, Point{10,10}, Point{3, 7}, Point{0,10} },
//                             {Point{3, 7}, Point{0,10}, Point{3, 3}, Point{0,0} } };
//      track.roadSegments = { {Point{0, 3}, Point{0, 0}, Point{7, 3}, Point{7,0} },
//                             {Point{7, 0}, Point{10,0}, Point{7, 7}, Point{10,7} },
//                             {Point{10, 7}, Point{10,10}, Point{3, 7}, Point{3,10} },
//                             {Point{3, 10}, Point{0,10}, Point{3, 3}, Point{0,3} } };
//      track.roadSegments = { { 0.0, 0.0, 7.0, 3.0, LeftToRight },
//                             { 7.0, 0.0, 10.0, 7.0, BottomToTop },
//                             { 3.0, 7.0, 10.0, 10.0, RightToLeft },
//                             { 0.0, 3.0, 3.0, 10.0, TopToBottom } };
      break;
//    case 1: // "L"-shaped track
//      track.playground = Box{ IV{ I{ 0, 19 }, I{ 0, 15 } } };
//      track.obstacles  = std::vector<Box>{ Box{ IV{ I{ 3, 6 }, I{ 3, 12 } } },
//                                           Box{ IV{ I{ 6, 16 }, I{ 3, 7 } } },
//                                           Box{ IV{ I{ 9, 19 }, I{ 10, 15 } } } };
//      track.waypoints  = std::vector<Point>{ Point{ 1.5, 1.5 },  Point{ 9.5, 1.5 },  Point{ 17.5, 1.5 },
//                                             Point{ 17.5, 8.5 }, Point{ 12.5, 8.5 }, Point{ 7.5, 8.5 },
//                                             Point{ 7.5, 11.0 }, Point{ 7.5, 13.5 }, Point{ 1.5, 13.5 } };
//
//      track.roadSegments = { { 0.0, 0.0, 16.0, 3.0, LeftToRight },
//                             { 16.0, 0.0, 19.0, 7.0, BottomToTop },
//                             { 9.0, 7.0, 19.0, 10.0, RightToLeft },
//                             { 6.0, 7.0, 9.0, 12.0, BottomToTop },
//                             { 3.0, 12.0, 9.0, 15.0, RightToLeft },
//                             { 0.0, 3.0, 3.0, 15.0, TopToBottom } };
      break;
      //    case 1: // "L"-shaped track
      //      track.playground = Box{ IV{ I{ 0, 19 }, I{ 0, 15 } } };
      //      track.obstacles  = std::vector<Box>{ Box{ IV{ I{ 3, 6 }, I{ 3, 12 } } },
      //                                           Box{ IV{ I{ 6, 16 }, I{ 3, 7 } } },
      //                                           Box{ IV{ I{ 9, 19 }, I{ 10, 15 } } } };
      //      track.waypoints  = std::vector<Point>{ Point{ 1.5, 1.5 },  Point{ 9.5, 1.5 },  Point{ 17.5, 1.5 },
      //                                             Point{ 17.5, 8.5 }, Point{ 12.5, 8.5 }, Point{ 7.5, 8.5 },
      //                                             Point{ 7.5, 11.0 }, Point{ 7.5, 13.5 }, Point{ 1.5, 13.5 } };
      //
      //      track.roadSegments = { { 0.0, 0.0, 16.0, 3.0, LeftToRight },
      //                             { 16.0, 0.0, 19.0, 7.0, BottomToTop },
      //                             { 9.0, 7.0, 19.0, 10.0, RightToLeft },
      //                             { 6.0, 7.0, 9.0, 12.0, BottomToTop },
      //                             { 3.0, 12.0, 9.0, 15.0, RightToLeft },
      //                             { 0.0, 3.0, 3.0, 15.0, TopToBottom } };
      break;
    default:
      throw std::logic_error( "Invalid trackID!" );
      break;
  }

  // Hard code starting position: take first waypoint, bloat it, if wanted
  // x, y, theta, tick, v
  Number wheelbase         = 1.0;
  Number bcVelocity        = 1;
  size_t bcMaxTurn         = 1;  // in theta buckets
  double bcStopZoneWidth   = 0.2; // for the car bc this is relative to the width of the road
  double bcBorderAngle     = 0.87;        /* 50° */
  Number acVelocity        = 2;           // 2,1;
  Number acLookahead       = 4.0;
  Number acScaling         = 0.55;  // 0.55,0.8;
  Number initialTheta      = 0.01;
  Point  initialPosition   = track.waypoints[0] + (track.waypoints[1] - track.waypoints[0])*0.5;
  Point  initialCarState   = Point( { initialPosition[0], initialPosition[1], initialTheta } );
  Point  initialState      = Point( { initialPosition[0], initialPosition[1], initialTheta, 0, bcVelocity } );
  IV initialValuations{ I{ initialPosition[0] }, I{ initialPosition[1] }, I{ initialTheta }, I{ 0 }, I{ bcVelocity } };

  // parse model
  hypro::ReachabilitySettings reachSettings;
  auto carModel          = modelGenerator::generateCarModel( theta_discretization, cycleTime, bcVelocity );
  carModel.setGlobalBadStates( track.createSafetySpecification() );
  reachSettings.timeStep = timeStepSize;
  {
    std::ofstream fs{ "car.model" };
    fs << hypro::toFlowstarFormat( carModel );
    fs.close();
  }

  auto purePursuitCtrl                 = new PurePursuitController(acVelocity, wheelbase, acLookahead, acScaling);
  purePursuitCtrl->track               = track;
  purePursuitCtrl->thetaDiscretization = theta_discretization;
  purePursuitCtrl->cycleTime           = cycleTime;
  purePursuitCtrl->lastWaypoint        = purePursuitCtrl->track.waypoints.begin();
  purePursuitCtrl->currentWaypoint     = std::next( purePursuitCtrl->lastWaypoint );

  auto randomCtrl = new RandomCarController(acVelocity, bcMaxTurn, theta_discretization);

  AbstractController<Point, Point>* advCtrl = purePursuitCtrl;

  // use first controller output to determine the starting location
  Point ctrlInput = advCtrl->generateInput( initialCarState );

  // determine correct starting location in two steps: (i) chose correct theta-bucket, (ii) modify delta_bucket to match
  // control output
  {
    auto startingLocationCandidates =
        getLocationForTheta( initialTheta, theta_discretization, carModel.getLocations() );
    if ( startingLocationCandidates.size() != 1 ) {
      throw std::logic_error( "Something went wrong during the starting location detection for the car model." );
    }
    auto* startingLocation = startingLocationCandidates.front();

    typename decltype( carModel )::locationConditionMap initialStates;
    initialStates.emplace( std::make_pair( startingLocation, hypro::conditionFromIntervals( initialValuations ) ) );
    carModel.setInitialStates( initialStates );
  }

  auto bc = simplexArchitectures::generateCarBaseController<hypro::HybridAutomaton<Number>>( theta_discretization, bcMaxTurn, bcStopZoneWidth,
                                                                bcBorderAngle, track.roadSegments, bcVelocity );
  auto& bcAtm = bc.mAutomaton;

  IV initialValuationsBC{ std::begin( initialValuations ), std::next( std::begin( initialValuations ), 2 ) };

  auto locCandidates = getLocationsForState( initialCarState, bcAtm );
  if ( locCandidates.size() != 1 ) {
    throw std::logic_error( "Something went wrong in the design of the BC." );
  }
  auto startingLocationBC = locCandidates.front();

  hypro::HybridAutomaton<Number>::locationConditionMap initialStatesBC;
  initialStatesBC.emplace( std::make_pair( startingLocationBC, hypro::conditionFromIntervals( initialValuationsBC ) ) );
  bcAtm.setInitialStates( initialStatesBC );

  // Automata compostion:
  auto                                                         mainLocations = carModel.getLocations();
  auto                                                         variables     = carModel.getVariables();
  std::map<std::string, std::vector<hypro::Location<Number>*>> variableMap;
  for ( const auto& var : variables ) {
    variableMap[var] = mainLocations;
  }

  Automaton automaton;
  Automaton simulationAutomaton;

  // Automata compostion:
  std::cout << "Env #locations:" << carModel.getLocations().size() << std::endl;
  std::cout << "BC #locations:" << bcAtm.getLocations().size() << std::endl;
  std::cout << "Start composition" << std::endl;
  auto start = std::chrono::steady_clock::now();
  automaton.addAutomaton( std::move( carModel ) );
  automaton.addAutomaton( std::move( bcAtm ) );
  // the car model dictates all dynamics
  automaton.makeComponentMaster( 0 );
  spdlog::trace("Composed automaton has {} variables",automaton.dimension());
  // hypro::parallelCompose( carModel, bcAtm, variableMap, reduceAutomaton );
  auto end = std::chrono::steady_clock::now();
  std::cout << "Composition finished" << std::endl;
  std::cout << "Elapsed time in milliseconds: "
            << std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count() << " ms" << std::endl;
  std::cout << "Combined #locations:" << automaton.getLocations().size() << std::endl;

  // create model for simulation: includes locations for all theta buckets, but only stop transitions
  // that do not change theta and reduce the speed to zero.
  // This should terminate the simulation after the first jump with finding a fixed point.
  {
    auto tmp = modelGenerator::generateCarModel( theta_discretization, cycleTime, bcVelocity, false );
    auto startingLocationCandidates = getLocationForTheta( initialTheta, theta_discretization, tmp.getLocations() );
    if ( startingLocationCandidates.size() != 1 ) {
      throw std::logic_error( "Something went wrong during the starting location detection for the car model." );
    }
    auto startingLocation = startingLocationCandidates.front();

    decltype( tmp )::locationConditionMap initialStates;
    initialStates.emplace( std::make_pair( startingLocation, hypro::conditionFromIntervals( initialValuations ) ) );
    tmp.setInitialStates( initialStates );
    // initialize simulation automaton
    simulationAutomaton.addAutomaton( std::move( tmp ) );
  }

  // reachability analysis settings, here only used for simulation
  auto settings                                                   = hypro::convert( reachSettings );
  settings.rStrategy().front().detectJumpFixedPoints              = true;
  settings.rStrategy().front().detectFixedPointsByCoverage        = false;
  settings.rStrategy().front().detectContinuousFixedPointsLocally = true;
  settings.rStrategy().front().detectZenoBehavior                 = true;
  settings.rStrategy().front().numberSetsForContinuousCoverage    = 2;
  settings.rFixedParameters().localTimeHorizon                    = 200;
  settings.rFixedParameters().jumpDepth                           = maxJumps;
  settings.rStrategy().begin()->aggregation                       = hypro::AGG_SETTING::AGG;

  // Storage for trained sets
  auto storagesettings = StorageSettings{ interesting_dimensions, Box{ track.playground.intervals() } };
  // filter only sets wherer the time is the tick-time
  // TODO get dimensions from some variables defined before
  Matrix constraints     = Matrix::Zero( 2, 5 );
  constraints( 0, tick ) = 1;
  constraints( 1, tick ) = -1;
  Vector constants       = Vector::Zero( 2 );
  constants << 0, -0;
  storagesettings.filter = hypro::Condition<Number>( constraints, constants );

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
  // the trainer runs on the combined automaton of base controller and car model
  Trainer trainer{ automaton, trainingSettings, storage };

  // set location-update functions
  auto updateFunctionExecutor = [&theta_discretization, &simulationAutomaton]( Point p, LocPtr l ) -> LocPtr {
    // TODO to make this more generic, we should keep a mapping from controller-output to actual state-space dimensions,
    // for now hardcode 0 (theta in ctrl-output)
    auto candidates = getLocationForTheta( p[0], theta_discretization, simulationAutomaton.getLocations() );
    if ( candidates.size() > 2 || candidates.empty() ) {
      std::cout << "Candidates:\n";
      for ( const auto* candidate : candidates ) {
        std::cout << candidate->getName() << "\n";
      }
      std::cout << std::flush;
      throw std::logic_error( "Number of control-location candidates (" + std::to_string( candidates.size() ) +
                              ") is invalid." );
    }
    return candidates.front();
  };
  auto updateFunctionSimulator = [&theta_discretization, &automaton]( Point p, LocPtr l ) -> LocPtr {
    // TODO to make this more generic, we should keep a mapping from controller-output to actual state-space dimensions,
    auto candidates = getLocationForTheta( p[0], theta_discretization, automaton.getLocations() );
    const std::regex oldSegmentZoneRegex("segment_([[:digit:]]+)_zone_([[:digit:]]+)$");
    std::smatch matches;
    const std::string tmp(l->getName());
    std::regex_search(tmp,matches,oldSegmentZoneRegex);
    std::string oldSegmentZoneSubstring = matches[0];

    LocPtr newLocation = nullptr;
    for(const auto* candidate : candidates) {
      if(candidate->getName().find(oldSegmentZoneSubstring) != std::string::npos) {
        //spdlog::trace("Found new location candidate: {}", candidate->getName());
        newLocation = candidate;
        break;
      }
    }
    if(newLocation == nullptr) {
      throw std::logic_error("New location not found");
    }
    return newLocation;
  };

  // the executor runs on the car model only
  assert( simulationAutomaton.getInitialStates().size() == 1 );
  Executor executor{ simulationAutomaton, simulationAutomaton.getInitialStates().begin()->first, initialState };
  executor.mSettings          = settings;
  executor.mExecutionSettings = ExecutionSettings{ 3, { theta, v } };
  executor.mPlot              = false;
  executor.mLocationUpdate    = updateFunctionExecutor;

  // monitor/simulator, runs on the car model
  Simulator sim{ automaton, settings, storage, controller_dimensions };
  sim.mLocationUpdate = updateFunctionSimulator;
  sim.mCycleTimeDimension = tick;
  sim.mCycleTime = cycleTime;
  sim.mObservationDimensions = std::vector<Eigen::Index>({x,y});
  // initialize simulator
  sim.mLastStates.emplace( std::make_pair( automaton.getInitialStates().begin()->first, std::set<Point>{ initialState } ) );

  if ( training ) {
    assert( automaton.getInitialStates().size() == 1 );
    auto initialStates = std::map<LocPtr, hypro::Condition<Number>>{};
    initialStates.emplace( std::make_pair(
        automaton.getInitialStates().begin()->first,
        hypro::Condition<Number>( widenSample( initialState, widening, trainingSettings.wideningDimensions ) ) ) );
    trainer.run( settings, initialStates );
     storage.plotCombined( "storage_post_initial_training_combined", true );
  }

  // for statistics: record in which iteration the base controller was needed
  std::vector<std::vector<bool>> baseControllerInvocations( 1 );
  std::vector<std::vector<bool>> computeAdaptation( 1 );
  std::size_t                    lapCounter = 0;
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
    if (alwaysUseBC) {
      Point bcInput = bc.generateInput( executor.mLastState );
      executeWithLapCount( executor, bcInput, lapCounter, track );
      if ( baseControllerInvocations.size() <= lapCounter ) {
        baseControllerInvocations.emplace_back( std::vector<bool>{} );
      }
      baseControllerInvocations[lapCounter].push_back( false );
      if ( computeAdaptation.size() <= lapCounter ) {
        computeAdaptation.emplace_back( std::vector<bool>{} );
      }
      computeAdaptation[lapCounter].push_back( false );
    } else if (alwaysUseAC) {
      executeWithLapCount( executor, advControllerInput, lapCounter, track );
      if ( baseControllerInvocations.size() <= lapCounter ) {
        baseControllerInvocations.emplace_back( std::vector<bool>{} );
      }
      baseControllerInvocations[lapCounter].push_back( false );
      if ( computeAdaptation.size() <= lapCounter ) {
        computeAdaptation.emplace_back( std::vector<bool>{} );
      }
      computeAdaptation[lapCounter].push_back( false );
    } else if ( advControllerSafe == hypro::TRIBOOL::TRUE ) {
      {
        std::stringstream ss;
        ss << advControllerInput;
        spdlog::debug( "Advanced controller is safe and traces end in known safe area, run with output {}", ss.str() );
      }
      executeWithLapCount( executor, advControllerInput, lapCounter, track );
      sim.update( advControllerInput, executor.mLastState );
      if ( baseControllerInvocations.size() <= lapCounter ) {
        baseControllerInvocations.emplace_back( std::vector<bool>{} );
      }
      baseControllerInvocations[lapCounter].push_back( false );
      if ( computeAdaptation.size() <= lapCounter ) {
        computeAdaptation.emplace_back( std::vector<bool>{} );
      }
      computeAdaptation[lapCounter].push_back( false );
    } else if ( advControllerSafe == hypro::TRIBOOL::FALSE ) {
      Point bcInput = bc.generateInput( executor.mLastState );
      {
        std::stringstream ss;
        ss << bcInput;
        spdlog::debug( "Advanced controller is unsafe, use base controller with output {}", ss.str() );
      }
      executeWithLapCount( executor, bcInput, lapCounter, track );
      sim.update( bcInput, executor.mLastState );
      advControllerUsed = false;
      if ( baseControllerInvocations.size() <= lapCounter ) {
        baseControllerInvocations.emplace_back( std::vector<bool>{} );
      }
      baseControllerInvocations[lapCounter].push_back( true );
      if ( computeAdaptation.size() <= lapCounter ) {
        computeAdaptation.emplace_back( std::vector<bool>{} );
      }
      computeAdaptation[lapCounter].push_back( false );
    } else {
      spdlog::debug( "Advanced controller is safe (bounded time), but traces end in unknown safety area" );
      bool allSafe = false;
      if ( training ) {
        spdlog::info( "Start training for {} locations", sim.unknownSamples.size() );
        allSafe = true;
        for ( const auto& [loc, setVector] : sim.unknownSamples ) {
          for ( const auto& set : setVector ) {
            locationConditionMap initialConfigurations{};
            auto                 setIntervals = set.intervals();
            setIntervals[0].bloat_by( widening );
            setIntervals[1].bloat_by( widening );
            initialConfigurations[loc] = hypro::Condition( setIntervals );
            auto safe                  = trainer.run( settings, initialConfigurations );
            {
              // TODO remove for performance reasons
              auto tmp = Representation(setIntervals);
              spdlog::debug( "Training result for location {} and set {}: {}", loc->getName(), tmp, safe );
            }
            allSafe = allSafe && safe;
          }
        }
        //{
        //  std::stringstream ss;
        //  std::size_t       l = std::to_string( iterations ).size();
        //  ss << std::setw( l ) << std::setfill( '0' ) << iteration_count;
        //  storage.plotCombined( "storage_post_training_" + ss.str() + "_combined" );
        //}
      }
      if ( !allSafe ) {
        Point bcInput = bc.generateInput( executor.mLastState );
        {
          std::stringstream ss;
          ss << bcInput;
          spdlog::debug( "Not all sets were safe (unbounded time), run base controller with output {}", ss.str() );
        }
        executeWithLapCount( executor, bcInput, lapCounter, track );
        sim.update( bcInput, executor.mLastState );
        advControllerUsed = false;
        if ( computeAdaptation.size() <= lapCounter ) {
          computeAdaptation.emplace_back( std::vector<bool>{} );
        }
        computeAdaptation[lapCounter].push_back( false );
        if ( baseControllerInvocations.size() <= lapCounter ) {
          baseControllerInvocations.emplace_back( std::vector<bool>{} );
        }
        baseControllerInvocations[lapCounter].push_back( true );
      } else {
        {
          std::stringstream ss;
          ss << advControllerInput;
          spdlog::debug( "All sets were safe (unbounded time), run advanced controller with output {}", ss.str() );
        }
        executeWithLapCount( executor, advControllerInput, lapCounter, track );
        sim.update( advControllerInput, executor.mLastState );
        if ( computeAdaptation.size() <= lapCounter ) {
          computeAdaptation.emplace_back( std::vector<bool>{} );
        }
        computeAdaptation[lapCounter].push_back( true );
        if ( baseControllerInvocations.size() <= lapCounter ) {
          baseControllerInvocations.emplace_back( std::vector<bool>{} );
        }
        baseControllerInvocations[lapCounter].push_back( false );
      }
    }

    std::stringstream ss;
    std::size_t       l = std::to_string( iterations ).size();
    ss << std::setw( l ) << std::setfill( '0' ) << iteration_count;
    if ( plotSets ) {
      storage.plotCombined( "car_storage_post_iteration_" + ss.str() + "_combined", false );
    } else {
      hypro::Plotter<Number>::getInstance().setFilename( "car_storage_post_iteration_" + ss.str() + "_combined" );
    }

    if ( plotPosition ) {
      auto fillsettings = hypro::Plotter<Number>::getInstance().settings();
      fillsettings.fill = true;
      if ( advControllerUsed ) {
        hypro::Plotter<Number>::getInstance().addPoint( executor.mLastState.projectOn( { 0, 1 } ),
                                                        hypro::plotting::colors[hypro::plotting::green], fillsettings );
      } else {
        hypro::Plotter<Number>::getInstance().addPoint(
            executor.mLastState.projectOn( { 0, 1 } ), hypro::plotting::colors[hypro::plotting::orange], fillsettings );
      }
    }
    if ( plotSets || plotPosition ) {
      hypro::Plotter<Number>::getInstance().plot2d( hypro::PLOTTYPE::png, false );
      hypro::Plotter<Number>::getInstance().clear();
    }

    if ( plotRaceTrack ) {
      auto& plt = hypro::Plotter<Number>::getInstance();
      plt.clear();
      plt.rSettings().keepAspectRatio = true;
      plt.rSettings().axes = false;
      plt.rSettings().grid = false;
      plt.rSettings().xPlotInterval   = carl::Interval<double>( track.playground.intervals()[0].lower() - 1.5,
                                                              track.playground.intervals()[0].upper() + 1.5 );
      plt.rSettings().yPlotInterval   = carl::Interval<double>( track.playground.intervals()[1].lower() - 1.5,
                                                              track.playground.intervals()[1].upper() + 1.5 );
      auto car                        = executor.mLastState.projectOn( { 0, 1, 2 } );
      auto color                      = advControllerUsed ? hypro::plotting::green : hypro::plotting::orange;
      track.addToPlotter( car, color );
      plt.setFilename( "racetrack_" + ss.str() );
      plt.plot2d( hypro::PLOTTYPE::png, true );
      plt.clear();
    }
  }
  for ( int i = 0; i < baseControllerInvocations.size(); ++i ) {
    std::size_t numberTrainings = std::count_if( std::begin( computeAdaptation[i] ), std::end( computeAdaptation[i] ),
                                                 []( bool val ) { return val; } );
    std::size_t numberBCInvocations =
        std::count_if( std::begin( baseControllerInvocations[i] ), std::end( baseControllerInvocations[i] ),
                       []( bool val ) { return val; } );
    spdlog::info( "Lap {} required {} iterations with {} BC-Invocations and {} successful trainings.", i,
                  baseControllerInvocations[i].size(), numberBCInvocations, numberTrainings );
  }
  // the training data is automatically stored in case the trainer runs out of scope
//  storage.plotCombined( "storage_post_execution_combined", true );
  return 0;
}
