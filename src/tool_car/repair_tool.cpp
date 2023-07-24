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

int main( int argc, char* argv[] ) {
  // settings
  std::size_t iterations{ 0 };
  std::size_t iteration_count{ 0 };
  std::size_t maxJumps             = 50;
  std::size_t theta_discretization = 36;
  Number      widening    = 1.0;

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

  // bc
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


  hypro::HybridAutomaton<Number> carModel2(carModel);
  hypro::HybridAutomaton<Number> specAtm2(specAtm);

  Automaton automaton;
  Automaton simAutomaton;

  // Automata compostion:
  automaton.addAutomaton( std::move( carModel ) );
  automaton.addAutomaton(std::move(specAtm));
  // the car model dictates all dynamics
  automaton.makeComponentMaster( 0 );
  automaton.makeComponentMasterForVariable(1, "timer");

  simAutomaton.addAutomaton( std::move( carModel2 ) );
  simAutomaton.addAutomaton(std::move(specAtm2));
  // the car model dictates all dynamics
  simAutomaton.makeComponentMaster( 0 );
  simAutomaton.makeComponentMasterForVariable(1, "timer");
  simAutomaton.setLazy(false);

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

//  if( extensiveInitialTraining ) {
    size_t segment_id = 0;
    spdlog::info("# road segments: {}", track.roadSegments.size());
//    for(auto segment : track.roadSegments) {
    auto segment = track.roadSegments[0];
    spdlog::info("Creating samples for segment {}", segment_id);
    auto trainingStates = generateTrainingSets(segment, segment_id, 2.0, theta_discretization, 2, automaton, bcVelocity, false);
    spdlog::info("{} samples generated; insert into storage", trainingStates.size());
    for (const auto& s : trainingStates) {
        for (const auto& l : s){
          auto box = Representation( l.second.getMatrix(), l.second.getVector() );
          storage.add(l.first->getName(), box);
        }
    }
    spdlog::info("training done");
    segment_id++;
//    }
//    storage.plotCombined( "storage_post_initial_training_combined", true ); //TODO Causes occasional segfaults
//  }


    auto carRepairExplorer = CarRepairExplorer(theta_discretization, bcMaxTurn, automaton, simAutomaton, settings, storage);

    // Create initial state for testing
    Point state = Point{40,22.5, 0, 0,bcVelocity,0};
    auto initialLoc = automaton.getLocation("theta-16_warning-C0");

    auto found = carRepairExplorer.findRepairSequence(initialLoc, state);
    spdlog::info("Repair sequence found: "+std::to_string(found));

    Point state2 = Point{40,20, 0, 0,bcVelocity,0};
    auto initialLoc2 = automaton.getLocation("theta-15_warning-C0");

    auto found2 = carRepairExplorer.findRepairSequence(initialLoc2, state2);
    spdlog::info("Repair sequence found: "+std::to_string(found2));


    //    storage.plotCombined( "storage_post_initial_training_combined", true ); //TODO Causes occasional segfaults

}