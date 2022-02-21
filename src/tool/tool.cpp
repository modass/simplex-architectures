/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 16.02.22.
 */



/*
 * Copyright (c) 2022.
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 28.10.21.
 */

#include "../types.h"
#include "../simulation/Simulator.h"
#include "../simulation/SamplingUtility.h"
#include "../controller/Controller.h"
#include "../utility/reachTreeUtility.h"
#include <hypro/algorithms/reachability/Reach.h>
#include <hypro/algorithms/reachability/handlers/badStateHandlers/ltiBadStateHandler.h>
#include <hypro/datastructures/Hyperoctree.h>
#include <hypro/datastructures/reachability/ReachTreev2Util.h>
#include <hypro/parser/antlr4-flowstar/ParserWrapper.h>
#include <hypro/paths.h>
#include <hypro/util/linearOptimization/Optimizer.h>
#include <hypro/util/plotting/Plotter.h>
#include <random>
#include <string>

/* GENERAL ASSUMPTIONS */
// The model does *not* contain timelocks

using namespace simplexArchitectures;

// global constants
static const std::vector<std::size_t> interesting_dimensions{ 0, 1, 3 };
static const std::vector<std::size_t> controller_dimensions{ 2 };


static Point generateObservation( const Simulator& sim ) {
  std::mt19937 generator;
  std::uniform_int_distribution<std::size_t> loc_dist{ 0, sim.mLastStates.size() - 1 };
  std::size_t chosenloc = loc_dist( generator );
  LocPtr locptr = std::next( sim.mLastStates.begin(), chosenloc )->first;
  std::uniform_int_distribution<std::size_t> point_dist{ 0, sim.mLastStates.at( locptr ).size() - 1 };
  return std::next( sim.mLastStates.at( locptr ).begin(), point_dist( generator ) )->projectOn( { 0, 1 } );
}

std::vector<hypro::Location<double> *> filterLocations(const std::vector<hypro::Location<double> *> &in, std::string filterExpression) {
  std::vector<hypro::Location<double> *> result;
  for (auto locPtr: in) {
    if (locPtr->getName().find(filterExpression) != std::string::npos) {
      result.push_back(locPtr);
    }
  }
  return result;
}

//std::vector<hypro::ReachTreeNode<Representation>> getTerminalNodes(const std::vector<hypro::ReachTreeNode<Representation>> &roots) {
//    std::vector<hypro::ReachTreeNode<Representation>> result;
//    for (const auto &root: roots) {
//        for (const auto &node: hypro::preorder(root)) {
//            if (node.getLocation()->getName().find("stopped") != std::string::npos && node.getTransition()->getSource()->getName().find("stopped") == std::string::npos) {
//                result.push_back(node);
//            }
//        }
//    }
//    return result;
//}

void plotOctree(const hypro::Hyperoctree<double> &octree, hypro::Plotter<double> &plt) {
  if (octree.isCovered()) {
    plt.addObject(octree.getContainer().projectOn({0, 1}).vertices(),
                  hypro::plotting::colors[hypro::plotting::green]);
  } else {
    if (!octree.getChildren().empty()) {
      for (const auto &child: octree.getChildren()) {
        plotOctree(child, plt);
      }
    } else {
      plt.addObject( octree.getContainer().projectOn( { 0, 1 } ).vertices(),
                    hypro::plotting::colors[hypro::plotting::red] );
    }
  }
}

void plotFlowpipes(
    const std::vector<hypro::ReachTreeNode<Representation>> &roots,
    const std::string &postfix) {
  auto &plt = hypro::Plotter<Number>::getInstance();
  plt.clear();
  plt.rSettings().overwriteFiles = true;
  plt.rSettings().cummulative = false;
  plt.rSettings().xPlotInterval = carl::Interval<double>(0.0, 0.8);
  plt.rSettings().yPlotInterval = carl::Interval<double>(0.0, 0.8);

  for (const auto &node : hypro::preorder(roots)) {
    auto color = (node.hasFixedPoint() == hypro::TRIBOOL::FALSE)
                     ? hypro::plotting::colors[hypro::plotting::orange]
                     : hypro::plotting::colors[hypro::plotting::blue];

        for ( const auto& segment : node.getFlowpipe() ) {
            plt.addObject(segment.projectOn({0, 1}).vertices(), color);
        }
    }
    plt.setFilename( "simplex_watertanks_loop_" + postfix );
    plt.plot2d( hypro::PLOTTYPE::png );
    plt.clear();
}

void reportIncompleteNodes(const std::vector<hypro::ReachTreeNode<Representation>> &roots) {
    bool allNodesFixedPoint = true;
    for ( const auto& node : hypro::preorder( roots ) ) {
        auto color = (node.hasFixedPoint() == hypro::TRIBOOL::FALSE) ?
                     hypro::plotting::colors[hypro::plotting::orange] :
                     hypro::plotting::colors[hypro::plotting::blue];
        if ( node.hasFixedPoint() == hypro::TRIBOOL::FALSE ) {
            std::cout << "Node " << node << " with path " << node.getPath() << " has no fixed point." << std::endl;
            allNodesFixedPoint = false;
        } else if ( node.hasFixedPoint() == hypro::TRIBOOL::NSET ) {
            std::cout << "Node " << node << " with path " << node.getPath() << " is indetermined" << std::endl;
            allNodesFixedPoint = false;
        }
    }
    if (allNodesFixedPoint) {
        std::cout << "All nodes have a fixed point." << std::endl;
    }
}

void updateOctree(std::map<const hypro::Location<Number>*, hypro::Hyperoctree<Number>> &octrees,
                  const std::vector<hypro::ReachTreeNode<Representation>> &roots) {
    //  constraints for cycle-time equals zero, encodes t <= 0 && -t <= -0
    hypro::matrix_t<Number> constraints = hypro::matrix_t<Number>::Zero(2, 5);
    hypro::vector_t<Number> constants = hypro::vector_t<Number>::Zero(2);
    constraints(0, 4) = 1;
    constraints( 1, 4 ) = -1;

    for ( const auto& r : roots ) {
        for ( const auto& node : hypro::preorder( r ) ) {
            for ( const auto& s : node.getFlowpipe() ) {
                // only store segments which contain states where the cycle time is zero
                if ( s.satisfiesHalfspaces( constraints, constants ).first != hypro::CONTAINMENT::NO ) {
                    octrees.at( node.getLocation() ).add( s.projectOn( interesting_dimensions ) );
                }
            }
        }
    }
}

#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnreachableCode"
int main(int argc, char *argv[]) {
  // read command line arguments
  if (argc != 2) {
    throw std::logic_error("You need to provide a filename as input.");
  }
  std::string filename = argv[1];
  std::cout << "Read input file: " << filename << std::endl;

  // settings
  std::size_t iterations{50};
  std::size_t iteration_count{0};
  std::size_t maxJumps = 50;
  Number widening = 0.01;
  bool training = true;
  // std::string filename{
  // "21_simplex_watertanks_deterministic_monitor_dbg_init_ticks.model" };

  // parse model
  auto [automaton, reachSettings] = hypro::parseFlowstarFile<Number>( filename );
  // reachability analysis settings
  auto settings = hypro::convert( reachSettings );
  settings.rStrategy().front().detectJumpFixedPoints = true;
  settings.rStrategy().front().detectFixedPointsByCoverage = true;
  settings.rStrategy().front().detectContinuousFixedPointsLocally = true;
  settings.rStrategy().front().numberSetsForContinuousCoverage = 2;
  settings.rFixedParameters().localTimeHorizon = 100;
  settings.rFixedParameters().jumpDepth = maxJumps;
  settings.rStrategy().begin()->aggregation = hypro::AGG_SETTING::AGG;
  // random controller
  Controller baseCtrl;
  Controller advCtrl;
  // monitor
  Simulator sim{ baseCtrl, advCtrl, automaton, settings };
  // we store one octree per location
  std::map<const hypro::Location<Number>*, hypro::Hyperoctree<Number>> octrees;
  for ( const auto* locPtr : automaton.getLocations() ) {
    // octree to store reachable sets, here fixed to [0,1] in each dimension
    octrees.emplace( locPtr, hypro::Hyperoctree<Number>( 2, 6, hypro::Box<Number>{ std::vector<carl::Interval<Number>>{ carl::Interval<Number>{ 0, 1 }, carl::Interval<Number>{ 0, 1 }, carl::Interval<Number>{ 0, 15 } } } ) );
  }
  // initialize system
  auto initialLocation = automaton.getInitialStates().begin()->first;
  auto initialValuation = automaton.getInitialStates().begin()->second.getInternalPoint().value();
  sim.mLastStates[initialLocation] = { initialValuation };
  // initial first reachability analysis for the initial point
  // new reachability analysis
  // reachability tree
  std::vector<hypro::ReachTreeNode<Representation>> roots;
  // update initial states - set to small box around sample
  auto intervals = widenSample( initialValuation, widening );
  auto initialBox = hypro::Condition<Number>{ intervals };
  hypro::HybridAutomaton<Number>::locationConditionMap initialStates;
  initialStates[initialLocation] = initialBox;
//  automaton.setInitialStates( initialStates );
  // store initial set in octree - we know its cycle-time is zero
//  octrees.at( initialLocation ).add( hypro::Box<Number>( intervals ) );
  // initialize reachtree
  roots = hypro::makeRoots<Representation>( automaton );
  // analysis
  auto reacher = hypro::reachability::Reach<Representation>( automaton, settings.fixedParameters(),
                                                            settings.strategy().front(), roots );
  std::cout << "Run initial analysis ... " << std::flush;
  auto result = reacher.computeForwardReachability();
  std::cout << "done, result: " << result << std::endl;
  // post processing
  if ( result != hypro::REACHABILITY_RESULT::SAFE ) {
    std::cout << "System is initially not safe, need to deal with this." << std::endl;
    exit( 1 );
  } else if (!hasFixedPoint(roots)) {
      std::cout << "System has no fixed point initially, need to deal with this." << std::endl;
      exit( 1 );
  }
  else {
      updateOctree(octrees, roots);
  }

  /* PLOTTING AFTER FIRST INIT */
    reportIncompleteNodes(roots);
    plotFlowpipes(roots, std::to_string( iteration_count ));

  // main loop which alternatingly invokes the controller and if necessary the analysis (training phase) for a bounded number of iterations
  while ( iteration_count < iterations ) {
    ++iteration_count;
    std::cout << "Iteration " << iteration_count << std::endl;

    // 1 simulation advanced controller, starting from initialvalue & location
    std::cout << "Start advanced controller simulation." << std::endl;
    bool advControllerSafe = sim.simulate( false );

    // if all safe & last point in reach set, pointify resulting set, update initialstate, update monitor (current point)
    if ( !advControllerSafe ) {
      // use base controller as fallback since the advanced controller seem to be unsafe
      std::cout << "Advanced controller is not safe, simulate base-controller, continue with next iteration." << std::endl;
      sim.simulate( true );
      sim.pointify();
      continue;
    }
    std::cout << "Advanced controller is safe, check if resulting simulation traces are in the octree." << std::endl;
    // at this point simulationSets is safe, the simulator knows the new initial state
    std::map<LocPtr, std::vector<Box>> unknownSamples;
    for ( const auto& r : sim.roots ) {
      for ( const auto& n : hypro::preorder( r ) ) {
        if ( n.isLeaf() && !octrees.at( n.getLocation() ).contains( n.getInitialSet().projectOn( interesting_dimensions ) ) ) {
          unknownSamples[n.getLocation()].push_back( n.getInitialSet() );
        }
      }
    }
    if ( unknownSamples.empty() ) {
      // if no new states have been discovered by simulation, continue, i.e., write new state from advanced controller as initial state of the simulator for the next iteration
      std::cout << "Advanced controller traces are in the octree." << std::endl;
      sim.pointify();
    } else {
      std::cout << "Advanced controller traces are not all in the octree." << std::endl;
      if ( training ) {
        std::cout << "Start training with " << unknownSamples.size() << " locations." << std::endl;
        //				std::cout << "Start training with samples: " << unknownSamples << std::endl;
        // if not in reach set: take new part, run base controller reachability analysis on this part

        // initialize reachtree
        roots.clear();
        std::size_t initialstateCount{ 0 };
        for ( const auto& [loc, boxes] : unknownSamples ) {
          for ( const auto& box : boxes ) {
            initialStates.clear();
            // initialStates[loc] = hypro::Condition<Number>( widenSample( sim.mPotentialNewState, widening ) );
            // set sample u-value to the correct value (corresponding to the value the base controller would have chosen, this can be obtained from the location name)
            auto newintervals = box.intervals();
            if ( loc->getName().find( "_on_" ) != std::string::npos ) {
              newintervals[2] = carl::Interval<Number>( 0.0002 );
            } else {
              newintervals[2] = carl::Interval<Number>( 0.0 );
            }
            initialStates[loc] = hypro::Condition<Number>( newintervals );
            automaton.setInitialStates( initialStates );
            auto tmp = hypro::makeRoots<Representation>( automaton );
            std::for_each( std::begin( tmp ), std::end( tmp ), [&roots]( auto&& node ) { roots.emplace_back( std::move( node ) ); } );
            ++initialstateCount;
          }
        }
        std::cout << "Set " << initialstateCount << " initial states for training, start analysis ... " << std::flush;

        // analysis
        auto reacher = hypro::reachability::Reach<Representation>( automaton, settings.fixedParameters(),
                                                                  settings.strategy().front(), roots );
        auto result = reacher.computeForwardReachability();
        std::cout << " done, result: " << result << std::endl;
        // post processing
        if ( result != hypro::REACHABILITY_RESULT::SAFE) {
            //// else switch to base controller, continue: simulate base controller, update sample (use monitor for this)
            // write new state - effectively simulates and uses base controller
            // TODO this is overly conservative, we could at least store results for samples (roots) that were safe.
            std::cout
                    << "Advanced controller is not safe on the long run, simulate base-controller, continue with next iteration."
                    << std::endl;
            sim.simulate(true);
            sim.pointify();
        } else if (!hasFixedPoint(roots)) {
            std::cout
                    << "No fixed point was found on the long run, simulate base-controller, continue with next iteration."
                    << std::endl;
            sim.simulate(true);
            sim.pointify();
        } else {
          //// if resulting analysis is safe, continue with advanced controller (pointify sample, update initial states), add to octree
          std::cout << "Advanced controller is safe on the long run, update octree with new sets, continue with next iteration." << std::endl;
          sim.pointify();
          updateOctree(octrees, roots);
        }
      } else {
        // we are not training and the sample is not yet known to be safe, switch to base controller
        std::cout << "We are not training, simulate base controller, continue with next iteration." << std::endl;
        sim.simulate( true );
        sim.pointify();
      }
    }

    plotFlowpipes(roots, std::to_string( iteration_count ));
  }

  return 0;
}
#pragma clang diagnostic pop
