//
// Created by bmaderbacher on 07.09.22.
//
#include "generateCarSpecification.h"

namespace simplexArchitectures {

template <typename HybridAutomaton>
HybridAutomaton generateCarSpecification( std::size_t theta_discretization,
                                          double warningZoneWidth, double maxIncursionTime,
                                          const std::vector<GeneralRoadSegment>& segments ){

  HybridAutomaton res;
  constexpr Eigen::Index         x     = 0;  // global position x
  constexpr Eigen::Index         y     = 1;  // global position y
  constexpr Eigen::Index         theta = 2;  // global theta heading
  constexpr Eigen::Index         timer = 3;  // spec timer
  constexpr Eigen::Index         C     = 4;  // constants

  std::vector<std::string> variableNames{ "x", "y", "theta", "timer" };
  res.setVariables( variableNames );


  const auto numberOfSegments = segments.size();
  std::map<std::tuple<std::size_t, std::size_t>, hypro::Location<double>*> buckets;

  Matrix flowmatrixTimerRunning = Matrix::Zero( variableNames.size() + 1, C + 1 );
  flowmatrixTimerRunning(timer, C) = 1.0;
  Matrix flowmatrixTimerPaused = Matrix::Zero( variableNames.size() + 1, C + 1 );

  // create locations
  for ( std::size_t is = 0; is < numberOfSegments; ++is ) {
    auto segment = segments[is];

    auto locLeft = res.createLocation();
    buckets.emplace( std::make_tuple( is, 0 ), locLeft );
    locLeft->setName( "warning-L" + std::to_string( is ) );
    auto invLeft = leftWarningInvariant( segment, warningZoneWidth );
    locLeft->setInvariant( invLeft );
    locLeft->setFlow( hypro::linearFlow<double>( flowmatrixTimerRunning ) );
    {
      auto stop = locLeft->createTransition( locLeft );
      stop->addLabel( hypro::Label( "stop" ) );
      for ( std::size_t it = 0; it < theta_discretization; ++it ) {
        // theta change
        auto changeTheta = locLeft->createTransition( locLeft );
        // label for controller synchronisation
        changeTheta->addLabel( hypro::Label( "set_theta_" + std::to_string( it ) ) );
      }
    }

    auto locCenter = res.createLocation();
    buckets.emplace( std::make_tuple( is, 1 ), locCenter );
    locCenter->setName( "warning-C"+std::to_string( is ));
    locCenter->setInvariant(centerWarningInvariant(segment, warningZoneWidth));
    locCenter->setFlow( hypro::linearFlow<double>( flowmatrixTimerPaused ) );

    {
      auto stop = locCenter->createTransition( locCenter );
      stop->addLabel( hypro::Label( "stop" ) );
      for ( std::size_t it = 0; it < theta_discretization; ++it ) {
        // theta change
        auto changeTheta = locCenter->createTransition( locCenter );
        // label for controller synchronisation
        changeTheta->addLabel( hypro::Label( "set_theta_" + std::to_string( it ) ) );
      }
    }

    auto locRight = res.createLocation();
    buckets.emplace( std::make_tuple( is, 2 ), locRight );
    locRight->setName( "warning-R"+std::to_string( is ));
    auto invRight = rightWarningInvariant(segment, warningZoneWidth);
    locRight->setInvariant(invRight);
    locRight->setFlow( hypro::linearFlow<double>( flowmatrixTimerRunning ) );

    {
      auto stop = locRight->createTransition( locRight );
      stop->addLabel( hypro::Label( "stop" ) );
      for ( std::size_t it = 0; it < theta_discretization; ++it ) {
        // theta change
        auto changeTheta = locRight->createTransition( locRight );
        // label for controller synchronisation
        changeTheta->addLabel( hypro::Label( "set_theta_" + std::to_string( it ) ) );
      }
    }
  }


  // create transitions between zones
  for ( auto& [key, source] : buckets ) {
    auto& [segmentId, zone] = key;
    auto segment            = segments[segmentId];

    switch (zone) {
      case 0: //left
      {
        auto centerLocation  = buckets[std::make_tuple( segmentId, 1 )];
        auto a               = segment.getWarningLeftStartRight(warningZoneWidth);
        auto b               = segment.getWarningLeftEndRight(warningZoneWidth);
        generateWarningCrossingTransition(source, centerLocation, a, b, theta_discretization, true);
        break;
      }
      case 1: //center
      {
        auto leftLocation  = buckets[std::make_tuple( segmentId, 0 )];
        auto aL               = segment.getWarningLeftEndRight(warningZoneWidth);
        auto bL               = segment.getWarningLeftStartRight(warningZoneWidth);
        generateWarningCrossingTransition(source, leftLocation, aL, bL, theta_discretization, false);

        auto rightLocation  = buckets[std::make_tuple( segmentId, 2 )];
        auto aR               = segment.getWarningRightStartLeft(warningZoneWidth);
        auto bR               = segment.getWarningRightEndLeft( warningZoneWidth );
        generateWarningCrossingTransition(source, rightLocation, aR, bR, theta_discretization, false);
        break;
      }
      case 2: //right
      {
        auto centerLocation  = buckets[std::make_tuple( segmentId, 1 )];
        auto a               = segment.getWarningRightEndLeft( warningZoneWidth );
        auto b               = segment.getWarningRightStartLeft(warningZoneWidth);
        generateWarningCrossingTransition(source, centerLocation, a, b, theta_discretization, true);
        break;
      }
    }
  }


    // segment changes
    if ( numberOfSegments > 1 ) {
      for ( auto segmentId = 0; segmentId < numberOfSegments; segmentId++ ) {
        auto segment = segments[segmentId];
        auto nextSegmentId     = segmentId < numberOfSegments - 1 ? segmentId + 1 : 0;
        auto previousSegmentId = segmentId > 0 ? segmentId - 1 : numberOfSegments - 1;

        for ( auto thisZoneId = 0; thisZoneId < 3; thisZoneId++ ) {
          auto targetZoneId = thisZoneId;
            auto thisLocation = buckets[std::tuple( segmentId, thisZoneId )];

            auto nextLocation = buckets[std::tuple( nextSegmentId, targetZoneId )];
            generateWarningCrossingTransition( thisLocation, nextLocation, segment.endRight, segment.endLeft,
                                        theta_discretization, false );

            auto previousLocation = buckets[std::tuple( previousSegmentId, targetZoneId )];
            generateWarningCrossingTransition( thisLocation, previousLocation, segment.startLeft, segment.startRight,
                                        theta_discretization, false );
        }
      }
    }

    Matrix constraints  = Matrix::Zero( 1, variableNames.size() );
    Vector constants    = Vector::Zero( 1 );

    constraints( 0, timer ) = -1;
    constants( 0 )      = -maxIncursionTime;

    res.addGlobalBadStates({constraints, constants});

    return res;

}



template<typename Location>
void generateWarningCrossingTransition( Location* origin, Location* target, Point borderA,
                                 Point borderB, std::size_t theta_discretization, bool resetClock) {
  constexpr Eigen::Index         x     = 0;  // global position x
  constexpr Eigen::Index         y     = 1;  // global position y
  constexpr Eigen::Index         theta = 2;  // global theta heading
  constexpr Eigen::Index         timer = 3;  // spec clock
  constexpr Eigen::Index         C     = 4;  // constants

  std::vector<std::string> variableNames{ "x", "y", "theta", "timer" };

  auto [angleLower, angleUpper] = crossingInterval(borderA, borderB, theta_discretization);

  //  spdlog::info("Cross from {} to {}. Angles: lower: {}, upper: {}", origin->getName(), target->getName(), angleLower, angleUpper);

  Matrix reset_matrix        = Matrix::Identity( variableNames.size(), variableNames.size() );
  Vector reset_vector        = Vector::Zero( variableNames.size() );
  reset_matrix( timer, timer ) = 0;

  if (angleLower < angleUpper) {
    auto transition = origin->createTransition( target );
    auto guard           = target->getInvariant();
    Matrix constraints      = Matrix::Zero( 2, variableNames.size() );
    Vector constants        = Vector::Zero( 2 );
    constraints( 0, theta ) = 1;
    constraints( 1, theta ) = -1;
    constants << angleUpper, -angleLower;
    guard.addConstraints( {constraints, constants} );
    transition->setGuard( guard );
    if(resetClock) {
      transition->setReset({reset_matrix, reset_vector});
    }
  } else {
    //create two transitions, because the theta interval wraps around zero
    {
      auto transition = origin->createTransition( target );
      auto guard      = target->getInvariant();
      Matrix constraints      = Matrix::Zero( 1, variableNames.size() );
      Vector constants        = Vector::Zero( 1 );
      constraints( 0, theta ) = -1;
      constants << -angleLower;
      guard.addConstraints( { constraints, constants } );
      transition->setGuard( guard );
      if(resetClock) {
        transition->setReset({reset_matrix, reset_vector});
      }
    }

    {
      auto transition = origin->createTransition( target );
      auto guard      = target->getInvariant();
      Matrix constraints      = Matrix::Zero( 1, variableNames.size() );
      Vector constants        = Vector::Zero( 1 );
      constraints( 0, theta ) = 1;
      constants << angleUpper;
      guard.addConstraints( { constraints, constants } );
      transition->setGuard( guard );
      if(resetClock) {
        transition->setReset({reset_matrix, reset_vector});
      }
    }
  }
}

} // namespace
