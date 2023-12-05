//
// Created by bmaderbacher on 07.09.22.
//
#include "generateCarBaseController.h"

namespace simplexArchitectures {

//template<typename HybridAutomaton>
//CarBaseController<HybridAutomaton> generateCarBaseController( std::size_t theta_discretization, size_t maxTurn, double stopZoneWidth,
//                                  double borderAngle, const std::vector<GeneralRoadSegment>& segments,
//                                  double velocity ) {
//
//  HybridAutomaton res;
//  constexpr Eigen::Index         x     = 0;  // global position x
//  constexpr Eigen::Index         y     = 1;  // global position y
//  constexpr Eigen::Index         theta = 2;  // global theta heading
//  constexpr Eigen::Index         C     = 3;  // constants
//
//  std::vector<std::string> variableNames{ "x", "y", "theta" };
//  res.setVariables( variableNames );
//
//  const auto numberOfSegments = segments.size();
//  std::map<std::tuple<std::size_t, std::size_t>, hypro::Location<double>*> buckets;
//
//  // create locations
//  for ( std::size_t is = 0; is < numberOfSegments; ++is ) {
//    auto segment = segments[is];
//
//    auto locLeft = res.createLocation();
//    buckets.emplace( std::make_tuple( is, 0 ), locLeft );
//    locLeft->setName( "segment-" + std::to_string( is ) + "-zone-0");
//    locLeft->setInvariant(segment.getLeftZoneInvariant(stopZoneWidth));
//
//    auto locCenter = res.createLocation();
//    buckets.emplace( std::make_tuple( is, 1 ), locCenter );
//    locCenter->setName( "segment-" + std::to_string( is ) + "-zone-1");
//    locCenter->setInvariant(segment.getCenterZoneInvariant(stopZoneWidth));
//
//    auto locRight = res.createLocation();
//    buckets.emplace( std::make_tuple( is, 2 ), locRight );
//    locRight->setName( "segment-" + std::to_string( is ) + "-zone-2");
//    locRight->setInvariant(segment.getRightZoneInvariant(stopZoneWidth));
//  }
//
//
//  // create output transitions and transitions between zones
//  for ( auto& [key, source] : buckets ) {
//    auto& [segmentId, zone] = key;
//    auto segment            = segments[segmentId];
//
//    switch (zone) {
//      case 0: //left
//      {
//        auto centerLocation  = buckets[std::make_tuple( segmentId, 1 )];
//        auto a               = segment.getCenterStartLeft( stopZoneWidth );
//        auto b               = segment.getCenterEndLeft( stopZoneWidth );
//        generateCrossingTransition(source, centerLocation, a, b, theta_discretization);
//        break;
//      }
//      case 1: //center
//      {
//        auto leftLocation  = buckets[std::make_tuple( segmentId, 0 )];
//        auto aL               = segment.getCenterEndLeft( stopZoneWidth );
//        auto bL               = segment.getCenterStartLeft( stopZoneWidth );
//        generateCrossingTransition(source, leftLocation, aL, bL, theta_discretization);
//
//        auto rightLocation  = buckets[std::make_tuple( segmentId, 2 )];
//        auto aR               = segment.getCenterStartRight( stopZoneWidth );
//        auto bR               = segment.getCenterEndRight( stopZoneWidth );
//        generateCrossingTransition(source, rightLocation, aR, bR, theta_discretization);
//        break;
//      }
//      case 2: //right
//      {
//        auto centerLocation  = buckets[std::make_tuple( segmentId, 1 )];
//        auto a               = segment.getCenterEndRight( stopZoneWidth );
//        auto b               = segment.getCenterStartRight( stopZoneWidth );
//        generateCrossingTransition(source, centerLocation, a, b, theta_discretization);
//        break;
//      }
//    }
//
//    double theta_increment = ( 2 * M_PI ) / double( theta_discretization );
//    double segmentAngle = segment.getSegmentAngle();
//
//    // outputs
//    double angle;
//    if ( zone == 1 ) {
//      angle = segmentAngle;
//    } else if ( zone == 0 ) {
//      angle = normalizeAngle( segmentAngle - borderAngle );
//    } else if ( zone == 2 ) {
//      angle = normalizeAngle( segmentAngle + borderAngle );
//    }
//
//    auto targetThetaBucket = getThetaBucket( angle, theta_discretization );
//
//    double theta_min = 0.0;
//    double theta_max = theta_increment;
//
//    for (size_t t = 0; t < theta_discretization; t++) {
//      auto differenceLeft = targetThetaBucket >= t ? targetThetaBucket - t : theta_discretization + targetThetaBucket - t;
//      auto differenceRight = t >= targetThetaBucket ? t - targetThetaBucket : theta_discretization + t - targetThetaBucket;
//
//      auto maxStopDifference = 1; //theta_discretization/8;
//      if (zone == 1 && (differenceLeft <= maxStopDifference || differenceRight <= maxStopDifference)) {
//        auto stopTrans = source->createTransition( source );
//        stopTrans->addLabel( hypro::Label( "stop" ) );
//
//        Matrix guard_constraints      = Matrix::Zero( 2, variableNames.size() );
//        Vector guard_constants        = Vector::Zero( 2 );
//        guard_constraints( 0, theta ) = 1;
//        guard_constraints( 1, theta ) = -1;
//        guard_constants << theta_max, -theta_min;
//
//        stopTrans->setGuard( { guard_constraints, guard_constants } );
//      } else if (differenceLeft == 0) {
//        auto newThetaBucket = targetThetaBucket;
//        auto thetaChange    = source->createTransition( buckets[std::make_tuple( segmentId, zone )] );
//        thetaChange->addLabel( hypro::Label( "set_theta_" + std::to_string( newThetaBucket ) ) );
//
//        Matrix guard_constraints  = Matrix::Zero( 2, variableNames.size() );
//        Vector guard_constants    = Vector::Zero( 2 );
//        guard_constraints( 0, theta ) = 1;
//        guard_constraints( 1, theta ) = -1;
//        guard_constants << theta_max, -theta_min;
//
//        thetaChange->setGuard({guard_constraints, guard_constants});
//
//      } else if (differenceLeft <= differenceRight) {
//        auto turn = std::min(maxTurn, differenceLeft);
//        auto newThetaBucket = t + turn >= theta_discretization ? t + turn - theta_discretization : t + turn;
//        auto thetaChange    = source->createTransition( buckets[std::make_tuple( segmentId, zone )] );
//        thetaChange->addLabel( hypro::Label( "set_theta_" + std::to_string( newThetaBucket ) ) );
//
//        Matrix guard_constraints  = Matrix::Zero( 2, variableNames.size() );
//        Vector guard_constants    = Vector::Zero( 2 );
//        guard_constraints( 0, theta ) = 1;
//        guard_constraints( 1, theta ) = -1;
//        guard_constants << theta_max, -theta_min;
//
//        thetaChange->setGuard({guard_constraints, guard_constants});
//
//      } else if (differenceRight < differenceLeft) {
//        auto turn = std::min(maxTurn, differenceRight);
//        auto newThetaBucket = turn > t ? theta_discretization+t-turn : t - turn;
//        auto thetaChange    = source->createTransition( buckets[std::make_tuple( segmentId, zone )] );
//        thetaChange->addLabel( hypro::Label( "set_theta_" + std::to_string( newThetaBucket ) ) );
//
//        Matrix guard_constraints  = Matrix::Zero( 2, variableNames.size() );
//        Vector guard_constants    = Vector::Zero( 2 );
//        guard_constraints( 0, theta ) = 1;
//        guard_constraints( 1, theta ) = -1;
//        guard_constants << theta_max, -theta_min;
//
//        thetaChange->setGuard({guard_constraints, guard_constants});
//      }
//
//      theta_min += theta_increment;
//      theta_max += theta_increment;
//    }
//  }
//
//
//  // segment changes
//  if ( numberOfSegments > 1 ) {
//    for ( auto segmentId = 0; segmentId < numberOfSegments; segmentId++ ) {
//      auto segment = segments[segmentId];
//      auto nextSegmentId     = segmentId < numberOfSegments - 1 ? segmentId + 1 : 0;
//      auto previousSegmentId = segmentId > 0 ? segmentId - 1 : numberOfSegments - 1;
//
//      for ( auto thisZoneId = 0; thisZoneId < 3; thisZoneId++ ) {
//        for ( auto targetZoneId = 0; targetZoneId < 3; targetZoneId++ ) {
//          auto thisLocation = buckets[std::tuple( segmentId, thisZoneId )];
//
//          auto nextLocation = buckets[std::tuple( nextSegmentId, targetZoneId )];
//          generateCrossingTransition( thisLocation, nextLocation, segment.endRight, segment.endLeft,
//                                      theta_discretization );
//
//          auto previousLocation = buckets[std::tuple( previousSegmentId, targetZoneId )];
//          generateCrossingTransition( thisLocation, previousLocation, segment.startLeft, segment.startRight,
//                                      theta_discretization );
//        }
//      }
//    }
//  }
//
//  CarBaseController<HybridAutomaton> result{velocity, maxTurn, stopZoneWidth, borderAngle, theta_discretization};
//  result.segments = segments;
//  result.mAutomaton = res;
//
//  return result;
//}


template<typename HybridAutomaton>
CarBaseController<HybridAutomaton> generateCarBaseController( std::size_t theta_discretization, size_t maxTurn, double stopZoneWidth,
                                                              double borderAngle, const std::vector<GeneralRoadSegment>& segments,
                                                              double velocity ) {

  HybridAutomaton res;
  constexpr Eigen::Index         x     = 0;  // global position x
  constexpr Eigen::Index         y     = 1;  // global position y
  constexpr Eigen::Index         theta = 2;  // global theta heading
  constexpr Eigen::Index         C     = 3;  // constants

  std::vector<std::string> variableNames{ "x", "y", "theta" };
  res.setVariables( variableNames );

  const auto numberOfSegments = segments.size();
  std::map<std::tuple<std::size_t, std::size_t>, hypro::Location<double>*> buckets;

  auto bcLocation = res.createLocation();
  bcLocation->setName( "bc");

  Matrix constraints  = Matrix::Zero( 2, variableNames.size() );
  Vector constants    = Vector::Zero( 2 );
  constraints( 0, theta ) = 1;
  constants(0) = 2 * M_PI;
  constraints( 1, theta ) = -1;
  constants(1) = 0;

  bcLocation->setInvariant({constraints, constants});

  for ( std::size_t segmentId = 0; segmentId < numberOfSegments; ++segmentId ) {
    auto segment = segments[segmentId];

    for ( std::size_t zone = 0; zone < 3; ++zone) {


      double theta_increment = ( 2 * M_PI ) / double( theta_discretization );
      double segmentAngle = segment.getSegmentAngle();


      double angle;
      hypro::Condition<Number> segmentInv;
      if ( zone == 1 ) {
        angle = segmentAngle;
      } else if ( zone == 0 ) {
        angle = normalizeAngle( segmentAngle - borderAngle );
      } else if ( zone == 2 ) {
        angle = normalizeAngle( segmentAngle + borderAngle );
      }

      auto targetThetaBucket = getThetaBucket( angle, theta_discretization );

      double theta_min = 0.0;
      double theta_max = theta_increment;

      for (size_t t = 0; t < theta_discretization; t++) {

        hypro::Condition<Number> segmentGuard;
        if ( zone == 1 ) {
          segmentGuard = segment.getCenterZoneHeadingGuard(stopZoneWidth, theta_min, theta_max);
        } else if ( zone == 0 ) {
          segmentGuard = segment.getLeftZoneHeadingGuard(stopZoneWidth, theta_min, theta_max);
        } else if ( zone == 2 ) {
          segmentGuard = segment.getRightZoneHeadingGuard(stopZoneWidth, theta_min, theta_max);
        }

        auto differenceLeft = targetThetaBucket >= t ? targetThetaBucket - t : theta_discretization + targetThetaBucket - t;
        auto differenceRight = t >= targetThetaBucket ? t - targetThetaBucket : theta_discretization + t - targetThetaBucket;

        auto maxStopDifference = 1; //theta_discretization/8;
        if (zone == 1 && (differenceLeft <= maxStopDifference || differenceRight <= maxStopDifference)) {
          auto stopTrans = bcLocation->createTransition( bcLocation );
          stopTrans->addLabel( hypro::Label( "stop" ) );
          stopTrans->setGuard( segmentGuard );

        } else if (differenceLeft == 0) {
          auto newThetaBucket = targetThetaBucket;
          auto thetaChange    = bcLocation->createTransition( bcLocation );
          thetaChange->addLabel( hypro::Label( "set_theta_" + std::to_string( newThetaBucket ) ) );
          thetaChange->setGuard( segmentGuard );

        } else if (differenceLeft <= differenceRight) {
          auto turn = std::min(maxTurn, differenceLeft);
          auto newThetaBucket = t + turn >= theta_discretization ? t + turn - theta_discretization : t + turn;
          auto thetaChange    = bcLocation->createTransition( bcLocation );
          thetaChange->addLabel( hypro::Label( "set_theta_" + std::to_string( newThetaBucket ) ) );
          thetaChange->setGuard( segmentGuard );

        } else if (differenceRight < differenceLeft) {
          auto turn = std::min(maxTurn, differenceRight);
          auto newThetaBucket = turn > t ? theta_discretization+t-turn : t - turn;
          auto thetaChange    = bcLocation->createTransition( bcLocation );
          thetaChange->addLabel( hypro::Label( "set_theta_" + std::to_string( newThetaBucket ) ) );
          thetaChange->setGuard( segmentGuard );
        }

        theta_min += theta_increment;
        theta_max += theta_increment;

      }

    }


  }

  CarBaseController<HybridAutomaton> result{velocity, maxTurn, stopZoneWidth, borderAngle, theta_discretization};
  result.segments = segments;
  result.mAutomaton = res;

  return result;
}


template<typename Location>
void generateCrossingTransition( Location* origin, Location* target, Point borderA,
                                  Point borderB, std::size_t theta_discretization) {
  constexpr Eigen::Index         x     = 0;  // global position x
  constexpr Eigen::Index         y     = 1;  // global position y
  constexpr Eigen::Index         theta = 2;  // global theta heading
  constexpr Eigen::Index         C     = 3;  // constants

  std::vector<std::string> variableNames{ "x", "y", "theta" };

  auto [angleLower, angleUpper] = crossingInterval(borderA, borderB, theta_discretization);

//  spdlog::info("Cross from {} to {}. Angles: lower: {}, upper: {}", origin->getName(), target->getName(), angleLower, angleUpper);

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
    }
  }
}

} // namespace
