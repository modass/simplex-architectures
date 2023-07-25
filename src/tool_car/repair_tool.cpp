//
// Created by bmaderbacher on 13.07.23.
//


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
#include "controller/StateActionMapController.h"
#include "ctrlConversion.h"
#include "model_generation/generateCarModel.h"
#include "model_generation/generateCarBaseController.h"
#include "model_generation/generateCarSpecification.h"
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
#include "training/CarTraining.h"

#include "simulation/CarRepairExplorer.h"
#include "utility/StateActionMap.h"

#include "../../racetracks/simpleL/bad_states.h"
#include "../../racetracks/simpleL/playground.h"
#include "../../racetracks/simpleL/segments.h"
#include "../../racetracks/simpleL/waypoints.h"

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
constexpr Eigen::Index                timer = 5;
constexpr Eigen::Index                C     = 6;
constexpr size_t model_dimensions           = 6;
static const std::vector<std::size_t> interesting_dimensions{ x, y, timer };
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
  std::size_t iterations{ 500 };
  std::size_t iteration_count{ 0 };
  std::size_t maxJumps             = 50;
  std::size_t theta_discretization = 36;
  Number      widening    = 0.25;

  RaceTrack track;
  track.playground   = createPlayground<Number>();
  track.obstacles    = createBadStates<hypro::HybridAutomaton<Number>>();
  track.roadSegments = createSegments<GeneralRoadSegment>();
  track.waypoints    = createWaypoints<Number>();


  // Hard code starting position: take first waypoint, bloat it, if wanted
  // x, y, theta, tick, v
  Number wheelbase         = 1.0;
  Number bcVelocity        = 1;
  size_t bcMaxTurn         = 1;  // in theta buckets
  double bcStopZoneWidth   = 0.2; // for the car bc this is relative to the width of the road
  double bcBorderAngle     = 0.87;        /* 50° */
  Number acVelocity        = 5;           // 2,1;
  double acMaxAngleChange  = (2*M_PI / theta_discretization) * bcMaxTurn + (2*M_PI / theta_discretization) * 0.5; //derived from bcMaxTurn
  Number acLookahead       = 10.0;
  Number acScaling         = 0.55;  // 0.55,0.8;
  Number initialTheta      = normalizeAngle( atan2((track.waypoints[1] - track.waypoints[0])[1], (track.waypoints[1] - track.waypoints[0])[0]) );
  Point  initialPosition   = track.waypoints[0] + (track.waypoints[1] - track.waypoints[0])*0.5;
  Point  initialCarState   = Point( { initialPosition[0], initialPosition[1], initialTheta } );
  Point  initialCarModelState      = Point( { initialPosition[0], initialPosition[1], initialTheta, 0, bcVelocity } );
  Point  initialState      = Point( { initialPosition[0], initialPosition[1], initialTheta, 0, bcVelocity, 0 } );
  IV initialCarModelValuations{ I{ initialPosition[0] }, I{ initialPosition[1] }, I{ initialTheta }, I{ 0 }, I{ bcVelocity } };
  IV initialValuations{ I{ initialPosition[0] }, I{ initialPosition[1] }, I{ initialTheta }, I{ 0 }, I{ bcVelocity }, I{ 0 } };
  Number                    timeStepSize{ 0.01 };
  Number                    cycleTime{ 0.1 };
  Number                    maxIncursionTime{ 2.0 };
  double                    relativeWarningBorderWidth{ 0.1 };
  std::string               storagefilename{ "storage_car_repair" };

  spdlog::set_level( spdlog::level::debug );

  auto& plt                       = hypro::Plotter<Number>::getInstance();
  plt.rSettings().overwriteFiles  = false;
  plt.rSettings().resolution      = std::pair<std::size_t, std::size_t>( 3000, 2000 );
  plt.rSettings().keepAspectRatio = true;
  plt.rSettings().plain           = true;


  hypro::ReachabilitySettings reachSettings;
  auto carModel          = modelGenerator::generateCarModel( theta_discretization, cycleTime, bcVelocity );
  carModel.setGlobalBadStates( track.createSafetySpecification() );
  reachSettings.timeStep = timeStepSize;

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
    initialStates.emplace( std::make_pair( startingLocation, hypro::conditionFromIntervals( initialCarModelValuations ) ) );
    carModel.setInitialStates( initialStates );
  }

  auto carModelSimulator          = modelGenerator::generateCarModel( theta_discretization, cycleTime, bcVelocity, false );
  carModelSimulator.setGlobalBadStates( track.createSafetySpecification() );
  // determine correct starting location in two steps: (i) chose correct theta-bucket, (ii) modify delta_bucket to match
  // control output
  {
    auto startingLocationCandidates =
        getLocationForTheta( initialTheta, theta_discretization, carModelSimulator.getLocations() );
    if ( startingLocationCandidates.size() != 1 ) {
      throw std::logic_error( "Something went wrong during the starting location detection for the car model." );
    }
    auto* startingLocation = startingLocationCandidates.front();

    typename decltype( carModelSimulator )::locationConditionMap initialStates;
    initialStates.emplace( std::make_pair( startingLocation, hypro::conditionFromIntervals( initialCarModelValuations ) ) );
    carModelSimulator.setInitialStates( initialStates );
  }



  // spec
  auto specAtm = simplexArchitectures::generateCarSpecification<hypro::HybridAutomaton<Number>>(theta_discretization, relativeWarningBorderWidth, maxIncursionTime, track.roadSegments);
  auto initialSpecState = Point{initialCarState[0], initialCarState[1], initialCarState[2], 0};
  auto locCandidatesSpec= getLocationsForState( initialSpecState, specAtm );
  if ( locCandidatesSpec.size() != 1 ) {
    throw std::logic_error( "Something went wrong when initializing the specification." );
  }
  auto startingLocationSpec = locCandidatesSpec.front();
  hypro::HybridAutomaton<Number>::locationConditionMap initialStatesSpec;
  IV initialValuationsSpec{ std::begin( initialValuations ), std::next( std::begin( initialValuations ), 3 ) };
  initialStatesSpec.emplace( std::make_pair( startingLocationSpec, hypro::conditionFromIntervals( initialValuationsSpec ) ) );
  specAtm.setInitialStates( initialStatesSpec );

//  // Automata compostion:
//  auto                                                         mainLocations = carModel.getLocations();
//  auto                                                         variables     = carModel.getVariables();
//  std::map<std::string, std::vector<hypro::Location<Number>*>> variableMap;
//  for ( const auto& var : variables ) {
//    variableMap[var] = mainLocations;
//  }


  hypro::HybridAutomaton<Number> carModelTrace(carModel);
  hypro::HybridAutomaton<Number> specAtmTrace(specAtm);
  hypro::HybridAutomaton<Number> specAtmSimulator(specAtm);


  Automaton automaton;
  Automaton traceAutomaton;
  Automaton executorAutomaton;
  Automaton simulatorAutomaton;

  // Automata compostion:
  automaton.addAutomaton( std::move( carModel ) );
  automaton.addAutomaton(std::move(specAtm));
  // the car model dictates all dynamics
  automaton.makeComponentMaster( 0 );
  automaton.makeComponentMasterForVariable(1, "timer");

  traceAutomaton.addAutomaton( std::move( carModelTrace ) );
  traceAutomaton.addAutomaton(std::move( specAtmTrace ));
  // the car model dictates all dynamics
  traceAutomaton.makeComponentMaster( 0 );
  traceAutomaton.makeComponentMasterForVariable(1, "timer");
  traceAutomaton.setLazy(false);

  // create model for execution: includes locations for all theta buckets, but only stop transitions
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
    executorAutomaton.addAutomaton( std::move( tmp ) );

  }

  simulatorAutomaton.addAutomaton(std::move(carModelSimulator));
  simulatorAutomaton.addAutomaton(std::move(specAtmSimulator));


  // reachability analysis settings, here only used for simulation
  auto settings                                                   = hypro::convert( reachSettings );
  settings.rStrategy().front().detectJumpFixedPoints              = true;
  settings.rStrategy().front().detectFixedPointsByCoverage        = false;
  settings.rStrategy().front().detectContinuousFixedPointsLocally = true;
  settings.rStrategy().front().detectZenoBehavior                 = true;
//  settings.rStrategy().front().numberSetsForContinuousCoverage    = 2;
  settings.rFixedParameters().localTimeHorizon                    = 200;
  settings.rFixedParameters().jumpDepth                           = maxJumps;
  settings.rStrategy().begin()->aggregation                       = hypro::AGG_SETTING::AGG;

  // Storage for trained sets
  auto intervals = track.playground.intervals();
  auto storagesettings = StorageSettings{ interesting_dimensions, Box{ {intervals[0], intervals[1], I{0.0, maxIncursionTime}} } };
  storagesettings.treeDepth = 4;
  // filter only sets wherer the time is the tick-time
  // TODO get dimensions from some variables defined before
  Matrix constraints     = Matrix::Zero( 2, model_dimensions );
  constraints( 0, tick ) = 1;
  constraints( 1, tick ) = -1;
  Vector constants       = Vector::Zero( 2 );
  constants << 0, -0;
  storagesettings.filter = hypro::Condition<Number>( constraints, constants );

  auto             storage = Storage( storagefilename, storagesettings );
  auto stateActionMap = StateActionMap<Box, hypro::Label>();

//  if( extensiveInitialTraining ) {
    size_t segment_id = 0;
    spdlog::info("# road segments: {}", track.roadSegments.size());
    for(auto segment : track.roadSegments) {
    spdlog::info("Creating samples for segment {}", segment_id);
    auto trainingStates = generateTrainingSets(segment, segment_id, 2.0, theta_discretization, 2, automaton, bcVelocity, false);
    spdlog::info("{} samples generated; insert into storage", trainingStates.size());
    for (const auto& s : trainingStates) {
        for (const auto& l : s){
          auto box = Representation( l.second.getMatrix(), l.second.getVector() );
          storage.add(l.first->getName(), box);
          stateActionMap.add(l.first->getName(), box, hypro::Label("stop"));
        }
    }
    spdlog::info("training done");
    segment_id++;
    }
//    storage.plotCombined( "storage_post_initial_training_combined", true ); //TODO Causes occasional segfaults
//  }


    // set location-update functions
    auto updateFunctionExecutor = [&theta_discretization, &executorAutomaton]( Point p, LocPtr l ) -> LocPtr {
      // TODO to make this more generic, we should keep a mapping from controller-output to actual state-space dimensions,
      // for now hardcode 0 (theta in ctrl-output)
      auto candidates = getLocationForTheta( p[0], theta_discretization, executorAutomaton.getLocations() );
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
    auto updateFunctionSimulator = [&theta_discretization, &simulatorAutomaton]( Point p, LocPtr l ) -> LocPtr {
      // TODO to make this more generic, we should keep a mapping from controller-output to actual state-space dimensions,
      spdlog::trace( "Start location update." );
      // auto candidates = getLocationForTheta( p[0], theta_discretization, automaton.getLocations() );
      // spdlog::trace("Found {} locations with the correct theta bucket.", candidates.size());
      const std::regex oldSegmentZoneRegex( "warning-(L|C|R)([[:digit:]]+)$" );
      std::smatch      matches;
      const std::string tmp( l->getName() );
      std::regex_search( tmp, matches, oldSegmentZoneRegex );
      std::string oldSegmentZoneSubstring = matches[0];

      auto        thetaBucket  = getThetaBucket( p[0], theta_discretization );
      std::string locationName = "theta-" + std::to_string( thetaBucket ) + "_" + oldSegmentZoneSubstring;
      LocPtr newLocation = simulatorAutomaton.getLocation( locationName );  // this should use a (cashed) hash map for efficiency

      /*
      LocPtr newLocation = nullptr;
      for(const auto* candidate : candidates) {
        if(candidate->getName().find(oldSegmentZoneSubstring) != std::string::npos) {
          //spdlog::trace("Found new location candidate: {}", candidate->getName());
          newLocation = candidate;
          break;
        }
      }
       */
      if(newLocation == nullptr) {
        auto ls = simulatorAutomaton.getLocations();
        spdlog::trace("Number of locations:" + std::to_string(ls.size()));
        for (const auto* l : ls) {
          if (l->getName() == locationName){
            spdlog::trace("Location " + locationName + " found.");
            break;
          }
        }
        if ( simulatorAutomaton.getLocation( locationName ) == nullptr) {
          spdlog::trace("Still not there");
        } else {
          spdlog::trace("Now it exists");
        }
        throw std::logic_error("New location (" + locationName + ") not found");
      }
      spdlog::trace("Location update complete.");
      return newLocation;
    };

    // the executor runs on the car model only
    assert( executorAutomaton.getInitialStates().size() == 1 );
    Executor executor{ executorAutomaton, executorAutomaton.getInitialStates().begin()->first, initialCarModelState };
    executor.mSettings          = settings;
    executor.mExecutionSettings = ExecutionSettings{ 3, { theta, v } };
    executor.mPlot              = false;
    executor.mLocationUpdate    = updateFunctionExecutor;
    executor.mCycleTime = cycleTime;

    // monitor/simulator, runs on the car model
    Simulator sim{ simulatorAutomaton, settings, storage, controller_dimensions };
    sim.mLocationUpdate = updateFunctionSimulator;
    sim.mCycleTimeDimension = tick;
    sim.mCycleTime = cycleTime;
    sim.mObservationDimensions = std::vector<Eigen::Index>({x,y});
    // initialize simulator
    sim.mLastStates.emplace( simulatorAutomaton.getInitialStates().begin()->first, std::set<Point>{ initialState } );


    // ===== Advanced controller ======

    auto purePursuitCtrl                 = new PurePursuitController(acVelocity, wheelbase, acLookahead, acScaling, acMaxAngleChange);
    purePursuitCtrl->track               = track;
    purePursuitCtrl->thetaDiscretization = theta_discretization;
    purePursuitCtrl->cycleTime           = cycleTime;
    purePursuitCtrl->lastWaypoint        = purePursuitCtrl->track.waypoints.begin();
    purePursuitCtrl->currentWaypoint     = std::next( purePursuitCtrl->lastWaypoint );

    auto randomCtrl = new RandomCarController(acVelocity, bcMaxTurn, theta_discretization);

    AbstractController<Point, Point>* advCtrl = purePursuitCtrl;

    // use first controller output to determine the starting location
    Point ctrlInput = advCtrl->generateInput( initialCarState );

    // ===== Base controller ======

    auto carRepairExplorer = CarRepairExplorer(theta_discretization, bcMaxTurn, automaton, traceAutomaton, settings, storage, stateActionMap);
    auto bc = StateActionMapController(bcVelocity, theta_discretization, stateActionMap, automaton);


    // ================ MAIN LOOP ================

    // for statistics: record in which iteration the base controller was needed
    std::vector<std::vector<bool>> baseControllerInvocations( 1 );
    std::vector<std::vector<bool>> computeAdaptation( 1 );
    std::vector<std::vector<Point>> positionHistory( 1 );
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
        bool           trainingUsed      = false;


        if ( advControllerSafe == hypro::TRIBOOL::TRUE ) {
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

            spdlog::info( "Start training for {} locations", sim.unknownSamples.size() );
            trainingUsed = true;
            allSafe      = true;
            for ( const auto& [loc, setVector] : sim.unknownSamples ) {
              for ( const auto& set : setVector ) {
                locationConditionMap initialConfigurations{};
                auto                 setIntervals = set.intervals();
                setIntervals[0].bloat_by( widening );
                setIntervals[1].bloat_by( widening );
                auto safe = carRepairExplorer.findRepairSequence(loc, hypro::Condition( setIntervals ));
                {
                  // TODO remove for performance reasons
                  auto tmp = Representation(setIntervals);
                  spdlog::debug( "Training result for location {} and set {}: {}", loc->getName(), tmp, safe );
                }
                allSafe = allSafe && safe;
              }
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
            advControllerUsed = true;
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

        if ( positionHistory.size() <= lapCounter ) {
          positionHistory.emplace_back( std::vector<Point>{} );
        }
        positionHistory[lapCounter].push_back( executor.mLastState.projectOn({x,y}) );
    }


    // ==================== End of execution statistics and plots =================

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
    storage.plotCombined( "storage_post_execution_combined", false );
    //  auto& plt = hypro::Plotter<Number>::getInstance();
    plt.rSettings().keepAspectRatio = true;
    plt.rSettings().axes = false;
    plt.rSettings().grid = false;
    plt.rSettings().xPlotInterval   = carl::Interval<double>( track.playground.intervals()[0].lower() - 1.5,
                                                            track.playground.intervals()[0].upper() + 1.5 );
    plt.rSettings().yPlotInterval   = carl::Interval<double>( track.playground.intervals()[1].lower() - 1.5,
                                                            track.playground.intervals()[1].upper() + 1.5 );
    auto car                        = executor.mLastState.projectOn( { 0, 1, 2 } );
    auto color                      = hypro::plotting::orange;
    track.addToPlotter( car, color );
    hypro::Plotter<Number>::getInstance().plot2d( hypro::PLOTTYPE::png, false );

    plt.clear();

    auto lastPoint = initialState.projectOn({x,y});
    for ( int i = 0; i < baseControllerInvocations.size(); ++i ) {
        auto positionHistoryLap = positionHistory[i];
        auto baseControllerInvocationsLap = baseControllerInvocations[i];
        for (int j = 0; j < baseControllerInvocationsLap.size(); ++j) {
          std::vector<Point> line{lastPoint, positionHistoryLap[j]};
          if ( !baseControllerInvocationsLap[j] ) {
            hypro::Plotter<Number>::getInstance().addPolyline(line,
                                                               hypro::plotting::colors[hypro::plotting::green]);
          } else {
            hypro::Plotter<Number>::getInstance().addPolyline(line,
                                                               hypro::plotting::colors[hypro::plotting::orange]);
          }
          lastPoint = positionHistoryLap[j];
        }
    }

    track.addToPlotter( car, color );
    plt.setFilename( "race_history" );
    plt.plot2d( hypro::PLOTTYPE::png, true );


    return 0;


}