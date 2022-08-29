//
// Created by bmaderbacher on 15.03.22.
//

#include "Executor.h"
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

Point simplexArchitectures::Executor::execute(const Point& ctrlInput) {
  roots.clear();

  hypro::Point<Number> extendedState = mLastState;
  // if any control has been provided, set it here
  auto i = 0;
  for ( Eigen::Index d : mExecutionSettings.control_input_dimensions ) {
    extendedState[d] = ctrlInput[i];
    i++;
  }
  if(mLocationUpdate) {
    mLastLocation = mLocationUpdate(ctrlInput,mLastLocation);
  }
  {
    std::stringstream ss;
    ss << extendedState;
    spdlog::debug( "Run executor with initial state {} in location {}", ss.str(), mLastLocation->getName() );
  }
  // create intervals representing the initial state
  std::vector<carl::Interval<Number>> intervals;
  for ( Eigen::Index i = 0; i < extendedState.dimension(); ++i ) {
    intervals.emplace_back( carl::Interval<Number>( extendedState.at( i ) ) );
  }
  auto                                                          initialBox = hypro::Condition<Number>{ intervals };
  typename hypro::HybridAutomaton<Number>::locationConditionMap initialStates;
  initialStates[mLastLocation] = initialBox;
    mAutomaton.setInitialStates( initialStates );
    auto sampleRoots = hypro::makeRoots<Representation>( mAutomaton );
    // add roots for this sample to global reachtree
    for ( auto&& sr : sampleRoots ) {
      roots.emplace_back( std::move( sr ) );
    }

    mSettings.rFixedParameters().globalTimeHorizon = 0.2;
    mSettings.rFixedParameters().localTimeHorizon  = carl::convert<double, hypro::tNumber>( 1.1*mCycleTime );
    mSettings.rFixedParameters().jumpDepth =
        2 * std::ceil( mCycleTime / carl::convert<hypro::tNumber, double>( mSettings.strategy().front().timeStep ) );

    auto reacher = ReachabilityAnalyzer( mAutomaton, mSettings.fixedParameters(), mSettings.strategy().front(), roots );
    auto isSafe  = reacher.computeForwardReachability();

    if(mPlot) {
      auto& plt = hypro::Plotter<Number>::getInstance();
      plt.clear();
      plt.setFilename("executor");
      plt.rSettings().overwriteFiles = false;
      for(const auto& r : roots) {
        for(const auto& n : hypro::preorder(r)) {
          for(const auto& s: n.getFlowpipe()) {
            plt.addObject(s.projectOn(plt.settings().dimensions).vertices());
          }
        }
      }
      plt.plot2d(hypro::PLOTTYPE::png, true);
    }

    if ( isSafe != hypro::REACHABILITY_RESULT::SAFE ) {
      spdlog::warn( "Executor reports potentially unsafe state!" );
      exit( 0 );
    }

    spdlog::trace("Have computed {} flowpipes with {} segments", hypro::getNumberNodes(roots), hypro::getNumberSegments(roots));

    for ( auto& root : roots ) {
      cutoffControllerJumps( &root );
    }

    // Pick an artificial observation.
    std::map<LocPtr, Box> samplesBoxes;
    // create constraints which fix the time to the last tick
    Matrix constraints = Matrix::Zero( 2, mAutomaton.dimension() );
    Vector constants   = Vector::Zero( 2 );
    // tick
    constraints( 0, mExecutionSettings.clock_dimension ) = 1;
    constraints( 1, mExecutionSettings.clock_dimension ) = -1;
    constants( 0 )                                       = 0;
    constants( 1 )                                       = -0;
    // collect all leaf nodes that agree with the cycle time
    for ( auto& r : roots ) {
      for ( auto& n : hypro::preorder( r ) ) {
        if ( n.isLeaf() ) {
          // I don't think we really need this check. We only consider initial sets of nodes that where reached by
          // resetting the cLocPtrk to zero.
          auto [containment, result] = n.getInitialSet().satisfiesHalfspaces( constraints, constants );
          if ( containment != hypro::CONTAINMENT::NO ) {
            //                    std::cout << "[Simulator] New sample: " << result << std::endl;
            if ( samplesBoxes.find( n.getLocation() ) != samplesBoxes.end() ) {
                        samplesBoxes[n.getLocation()] = samplesBoxes[n.getLocation()].unite( result );
                    } else {
              samplesBoxes[n.getLocation()] = result;
            }
          }
        }
      }
    }
    // create an artificial observation
    if ( samplesBoxes.empty() ) {
      throw std::logic_error( "No samples available after execution" );
    }
    std::uniform_int_distribution<std::size_t> LocPtr_dist{ 0, samplesBoxes.size() - 1 };
    std::size_t                                chosenLocPtr = LocPtr_dist( mGenerator );
    LocPtr                                     location     = std::next( samplesBoxes.begin(), chosenLocPtr )->first;
    //Point observation = samplesBoxes.at( location ).vertices().front();  //.projectOn( { 0, 1 } );
    hypro::vector_t<Number> observation_raw = hypro::vector_t<Number>(samplesBoxes.at(location).dimension());
    for(Eigen::Index i = 0; i < samplesBoxes.at(location).dimension(); ++i) {
      observation_raw(i) = carl::center(samplesBoxes.at(location).intervals()[i]);
    }
    Point observation{observation_raw};

    mLastLocation = location;
    mLastState    = observation;
    {
      std::stringstream ss;
      ss.str( std::string() );
      ss << observation;
      spdlog::debug( "Executor has new state: {} in location {}", ss.str(), mLastLocation->getName() );
    }

    return observation;
}
