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

#include "../controller/Controller.h"
#include "../simulation/SamplingUtility.h"
#include "../simulation/Simulator_old.h"
#include "../types.h"
#include "../utility/reachTreeUtility.h"
#include "../utility/treeSerialization.h"
#include "../training/Trainer.h"

/* GENERAL ASSUMPTIONS */
// The model does *not* contain timelocks

using namespace simplexArchitectures;

using I = carl::Interval<Number>;
using IV = std::vector<I>;
using Box = hypro::Box<Number>;

// global constants
static const std::vector<std::size_t> interesting_dimensions{ 0, 1, 3 };
static const std::vector<std::size_t> controller_dimensions{ 2 };

static Point generateObservation( const Simulator &sim ) {
  std::mt19937                               generator;
  std::uniform_int_distribution<std::size_t> loc_dist{ 0, sim.mLastStates.size() - 1 };
  std::size_t                                chosenloc = loc_dist( generator );
  LocPtr                                     locptr    = std::next( sim.mLastStates.begin(), chosenloc )->first;
  std::uniform_int_distribution<std::size_t> point_dist{ 0, sim.mLastStates.at( locptr ).size() - 1 };
  return std::next( sim.mLastStates.at( locptr ).begin(), point_dist( generator ) )->projectOn( { 0, 1 } );
}

std::vector<hypro::Location<double> *> filterLocations( const std::vector<hypro::Location<double> *> &in,
                                                        std::string filterExpression ) {
  std::vector<hypro::Location<double> *> result;
  for ( auto locPtr : in ) {
    if ( locPtr->getName().find( filterExpression ) != std::string::npos ) {
      result.push_back( locPtr );
    }
  }
  return result;
}

// std::vector<hypro::ReachTreeNode<Representation>> getTerminalNodes(const
// std::vector<hypro::ReachTreeNode<Representation>> &roots) {
//     std::vector<hypro::ReachTreeNode<Representation>> result;
//     for (const auto &root: roots) {
//         for (const auto &node: hypro::preorder(root)) {
//             if (node.getLocation()->getName().find("stopped") != std::string::npos &&
//             node.getTransition()->getSource()->getName().find("stopped") == std::string::npos) {
//                 result.push_back(node);
//             }
//         }
//     }
//     return result;
// }

void plotOctree( const hypro::Hyperoctree<double> &octree, hypro::Plotter<double> &plt ) {
  if ( octree.isCovered() ) {
    plt.addObject( octree.getContainer().projectOn( { 0, 1 } ).vertices(),
                   hypro::plotting::colors[hypro::plotting::green] );
  } else {
    if ( !octree.getChildren().empty() ) {
      for ( const auto &child : octree.getChildren() ) {
        plotOctree( child, plt );
      }
    } else {
      plt.addObject( octree.getContainer().projectOn( { 0, 1 } ).vertices(),
                     hypro::plotting::colors[hypro::plotting::red] );
    }
  }
}

void plotFlowpipes( const std::vector<hypro::ReachTreeNode<Representation>> &roots, const std::string &postfix ) {
  auto &plt = hypro::Plotter<Number>::getInstance();
  plt.clear();
  plt.rSettings().overwriteFiles = true;
  plt.rSettings().cummulative    = false;
  plt.rSettings().xPlotInterval  = carl::Interval<double>( 0.0, 0.8 );
  plt.rSettings().yPlotInterval  = carl::Interval<double>( 0.0, 0.8 );

  for ( const auto &node : hypro::preorder( roots ) ) {
    auto color = ( node.hasFixedPoint() == hypro::TRIBOOL::FALSE ) ? hypro::plotting::colors[hypro::plotting::orange]
                                                                   : hypro::plotting::colors[hypro::plotting::blue];

    for ( const auto &segment : node.getFlowpipe() ) {
      plt.addObject( segment.projectOn( { 0, 1 } ).vertices(), color );
    }
  }
  plt.setFilename( "simplex_watertanks_loop_" + postfix );
  plt.plot2d( hypro::PLOTTYPE::png, true );
  plt.clear();
}

void reportIncompleteNodes( const std::vector<hypro::ReachTreeNode<Representation>> &roots ) {
  bool allNodesFixedPoint = true;
  for ( const auto &node : hypro::preorder( roots ) ) {
    auto color = ( node.hasFixedPoint() == hypro::TRIBOOL::FALSE ) ? hypro::plotting::colors[hypro::plotting::orange]
                                                                   : hypro::plotting::colors[hypro::plotting::blue];
    if ( node.hasFixedPoint() == hypro::TRIBOOL::FALSE ) {
      std::cout << "Node " << node << " with path " << node.getPath() << " has no fixed point." << std::endl;
      allNodesFixedPoint = false;
    } else if ( node.hasFixedPoint() == hypro::TRIBOOL::NSET ) {
      std::cout << "Node " << node << " with path " << node.getPath() << " is indetermined" << std::endl;
      allNodesFixedPoint = false;
    }
  }
  if ( allNodesFixedPoint ) {
    std::cout << "All nodes have a fixed point." << std::endl;
  }
}

void updateOctree( std::map<std::string, hypro::Hyperoctree<Number>>       &octrees,
                   const std::vector<hypro::ReachTreeNode<Representation>> &roots ) {
  //  constraints for cycle-time equals zero, encodes t <= 0 && -t <= -0
  hypro::matrix_t<Number> constraints = hypro::matrix_t<Number>::Zero( 2, 5 );
  hypro::vector_t<Number> constants   = hypro::vector_t<Number>::Zero( 2 );
  constraints( 0, 4 )                 = 1;
  constraints( 1, 4 )                 = -1;

  for ( const auto &r : roots ) {
    for ( const auto &node : hypro::preorder( r ) ) {
      for ( const auto &s : node.getFlowpipe() ) {
        // only store segments which contain states where the cycle time is zero
        if ( s.satisfiesHalfspaces( constraints, constants ).first != hypro::CONTAINMENT::NO ) {
          auto tmp = s.projectOn( interesting_dimensions );
          if ( !octrees.at( node.getLocation()->getName() ).contains( tmp ) ) {
            octrees.at( node.getLocation()->getName() ).add( tmp );
          }
        }
      }
    }
  }
}

#pragma clang diagnostic push
#pragma ide diagnostic   ignored "UnreachableCode"
int                      main( int argc, char *argv[] ) {
  // settings
  std::size_t iterations{ 5 };
  std::size_t iteration_count{ 0 };
  std::size_t maxJumps = 200;
  Number      widening = 0.01;
  bool        training = true;
  std::string modelfilename;
  std::string storagefilename;
  bool haveTreeFile = true;
  std::map<std::string,hypro::Hyperoctree<Number>> trees;

  CLI::App app{ "Training application for simplex architectures project." };
  app.add_option( "-m,--model", modelfilename, "Path to the model file" )->required()->check( CLI::ExistingFile );
  app.add_option( "-s,--storage", storagefilename, "Path to file with stored sets" )->required();
  app.add_option( "-i,--iterations", iterations, "Number of iterations/steps" )->check( CLI::PositiveNumber );
  app.add_flag("--learn", training, "If given, the method will try to add new initial sets to the safe area, if they are safe. Otherwise the analysis terminates?");
  CLI11_PARSE( app, argc, argv );
  // parse model
  auto [automaton, reachSettings] = hypro::parseFlowstarFile<Number>( modelfilename );
  // reachability analysis settings, here only used for simulation
  auto settings                                            = hypro::convert( reachSettings );
  settings.rStrategy().front().detectJumpFixedPoints       = true;
  settings.rStrategy().front().detectFixedPointsByCoverage = true;
  settings.rStrategy().front().detectContinuousFixedPointsLocally = true;
  settings.rStrategy().front().detectZenoBehavior                 = true;
  settings.rStrategy().front().numberSetsForContinuousCoverage    = 2;
  settings.rFixedParameters().localTimeHorizon                    = 100;
  settings.rFixedParameters().jumpDepth                           = maxJumps;
  settings.rStrategy().begin()->aggregation                       = hypro::AGG_SETTING::AGG;
  // random controller
  Controller baseCtrl;
  Controller advCtrl;
  // monitor
  Simulator sim{ baseCtrl, advCtrl, automaton, settings };

  // initial trainging, if required, otherwise just load the treefile and update the local variable (trees)
  // Storagesettings will be overidden if a file with data exists
  StorageSettings storageSettings{ { 0, 1, 4 }, Box{ IV{ I{ 0, 1 }, I{ 0, 1 }, I{ 0, 15 } } }, 2, 4 };
  TrainingSettings trainingSettings{ 1,
                                     INITIAL_STATE_HEURISTICS::SINGLE,
                                     { 0, 1 },
                                     Box{ IV{ I{ 0.2, 0.5 }, I{ 0.2, 0.5 }, I{ 0 }, I{ 0 }, I{ 0 } } },
                                     { 10, 10, 1, 1, 1 } };
  Trainer trainer{storagefilename,modelfilename,trainingSettings,storageSettings};
  if(training && !haveTreeFile) {
    trainer.run(settings,automaton.getInitialStates());
  }
  // update octrees - the trainer loads the tree-file
  trees = trainer.getTrainingData();

  // main loop which alternatingly invokes the controller and if necessary the analysis (training phase) for a bounded
  // number of iterations
  while ( iteration_count++ < iterations ) {
    spdlog::info( "Iteration {}", iteration_count );

    // 1 simulation advanced controller, starting from initialvalue & location
    spdlog::info( "Start advanced controller simulation." );
    bool advControllerSafe = sim.simulate( false );

    // if all safe & last point in reach set, pointify resulting set, update initialstate, update monitor (current
    // point)
    if ( !advControllerSafe ) {
      // use base controller as fallback since the advanced controller seem to be unsafe
      spdlog::info( "Advanced controller is not safe, simulate base-controller, continue with next iteration." );
      sim.simulate( true );
      sim.pointify();
      continue;
    }
    spdlog::info( "Advanced controller is safe, check if resulting simulation traces are in the octree." );
    // at this point simulationSets is safe, the simulator knows the new initial state
    std::map<LocPtr, std::vector<Box>> unknownSamples;
    for ( const auto &r : sim.roots ) {
      for ( const auto &n : hypro::preorder( r ) ) {
        if ( n.isLeaf() && !trees.at( n.getLocation()->getName() )
                                .contains( n.getInitialSet().projectOn( interesting_dimensions ) ) ) {
          unknownSamples[n.getLocation()].push_back( n.getInitialSet() );
        }
      }
    }
    if ( unknownSamples.empty() ) {
      // if no new states have been discovered by simulation, continue, i.e., write new state from advanced controller
      // as initial state of the simulator for the next iteration
      spdlog::info( "Advanced controller traces are in the octree." );
      sim.pointify();
    } else {
      spdlog::info( "Advanced controller traces are not all in the octree." );
      if ( training ) {
        spdlog::info( "Start training with {} locations.", unknownSamples.size() );

        std::size_t initialstateCount{ 0 };
        for ( const auto &[loc, boxes] : unknownSamples ) {
          for ( const auto &box : boxes ) {
            locationConditionMap newInitialStates;
            // set sample u-value to the correct value (corresponding to the value the base controller would have
            // chosen, this can be obtained from the location name)
            auto newintervals = box.intervals();
            if ( loc->getName().find( "_on_" ) != std::string::npos ) {
              newintervals[2] = carl::Interval<Number>( 0.0002 );
            } else {
              newintervals[2] = carl::Interval<Number>( 0.0 );
            }
            newInitialStates[loc] = hypro::Condition<Number>( newintervals );

            trainer.run(settings,newInitialStates);
            trees = trainer.getTrainingData();
          }
        }

        /*
        // analysis
        auto reacher = hypro::reachability::Reach<Representation>( automaton, settings.fixedParameters(),
                                                                   settings.strategy().front(), roots );
        auto result = reacher.computeForwardReachability();
        spdlog::info( "done." );
        // post processing
        if ( result != hypro::REACHABILITY_RESULT::SAFE ) {
          //// else switch to base controller, continue: simulate base controller, update sample (use monitor for this)
          // write new state - effectively simulates and uses base controller
          // TODO this is overly conservative, we could at least store results for samples (roots) that were safe.
          spdlog::info(
                                   "Advanced controller is not safe on the long run, simulate base-controller, continue with next "
                                                        "iteration." );
          sim.simulate( true );
          sim.pointify();
        } else if ( !hasFixedPoint( roots ) ) {
          spdlog::info(
                                   "No fixed point was found on the long run, simulate base-controller, continue with next iteration." );
          sim.simulate( true );
          sim.pointify();
        } else {
          //// if resulting analysis is safe, continue with advanced controller (pointify sample, update initial
          ///states), add to octree
          spdlog::info(
                                   "Advanced controller is safe on the long run, update octree with new sets, continue with next "
                                                        "iteration." );
          sim.pointify();
          updateOctree( trees, roots );
        }
         */
      } else {
        // we are not training and the sample is not yet known to be safe, switch to base controller
        spdlog::info( "We are not training, simulate base controller, continue with next iteration." );
        sim.simulate( true );
        sim.pointify();
      }
    }
  }
  // the training data is automatically stored in case the trainer runs out of scope
  return 0;
}
#pragma clang diagnostic pop
