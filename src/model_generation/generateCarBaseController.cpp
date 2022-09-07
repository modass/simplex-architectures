//
// Created by bmaderbacher on 07.09.22.
//
#include "generateCarBaseController.h"

namespace simplexArchitectures {

CarBaseController generateCarBaseController( std::size_t theta_discretization, size_t maxTurn, double stopZoneWidth,
                                  double borderAngle, const std::vector<GeneralRoadSegment>& segments,
                                  double velocity ) {

  hypro::HybridAutomaton<double> res;
  constexpr Eigen::Index         x     = 0;  // global position x
  constexpr Eigen::Index         y     = 1;  // global position y
  constexpr Eigen::Index         theta = 2;  // global position y
  constexpr Eigen::Index         C     = 3;  // constants

  std::vector<std::string> variableNames{ "x", "y", "theta" };
  res.setVariables( variableNames );

  const auto numberOfSegments = segments.size();
  std::map<std::tuple<std::size_t, std::size_t>, hypro::Location<double>*> buckets;

  // create locations
  for ( std::size_t is = 0; is < numberOfSegments; ++is ) {
    auto segment = segments[is];

    auto locLeft = res.createLocation();
    buckets.emplace( std::make_tuple( is, 0 ), locLeft );
    locLeft->setName( "segment_" + std::to_string( is ) + "_zone_0");
    locLeft->setInvariant(segment.getLeftZoneInvariant(stopZoneWidth));

    auto locCenter = res.createLocation();
    buckets.emplace( std::make_tuple( is, 1 ), locCenter );
    locCenter->setName( "segment_" + std::to_string( is ) + "_zone_1");
    locCenter->setInvariant(segment.getCenterZoneInvariant(stopZoneWidth));

    auto locRight = res.createLocation();
    buckets.emplace( std::make_tuple( is, 2 ), locRight );
    locRight->setName( "segment_" + std::to_string( is ) + "_zone_2");
    locRight->setInvariant(segment.getRightZoneInvariant(stopZoneWidth));
  }


  // create output transitions and transitions between zones
  for ( auto& [key, source] : buckets ) {
    auto& [segmentId, zone] = key;
    auto segment            = segments[segmentId];

    size_t leftZone    = zone - 1;
    bool   leftExists  = zone > 0;
    size_t rightZone   = zone + 1;
    bool   rightExists = zone < 2;

    // connections in the same segment
    if ( leftExists ) {
      auto leftLocation   = buckets[std::make_tuple( segmentId, leftZone )];
      auto leftTransition = source->createTransition( leftLocation );
      leftTransition->setGuard( leftLocation->getInvariant() );
    }

    if ( rightExists ) {
      auto rightLocation   = buckets[std::make_tuple( segmentId, rightZone )];
      auto rightTransition = source->createTransition( rightLocation );
      rightTransition->setGuard( rightLocation->getInvariant() );
    }

    // outputs
    double segmentAngle = segment.getSegmentAngle();
    double angle;
    if ( zone == 1 ) {
      angle = segmentAngle;
    } else if ( zone == 0 ) {
      angle = normalizeAngle( segmentAngle - borderAngle );
    } else if ( zone == 2 ) {
      angle = normalizeAngle( segmentAngle + borderAngle );
    }

    auto targetThetaBucket = getThetaBucket( angle, theta_discretization );

    double theta_increment      = ( 2 * M_PI ) / double( theta_discretization );
    double theta_min = 0.0;
    double theta_max = theta_increment;

    for (size_t t = 0; t < theta_discretization; t++) {
      auto differenceLeft = targetThetaBucket >= t ? targetThetaBucket - t : theta_discretization + targetThetaBucket - t;
      auto differenceRight = t >= targetThetaBucket ? t - targetThetaBucket : theta_discretization + t - targetThetaBucket;

      auto maxStopDifference = theta_discretization/8;
      if (zone == 1 && (differenceLeft <= maxStopDifference || differenceRight <= maxStopDifference)) {
        auto stopTrans = source->createTransition( source );
        stopTrans->addLabel( hypro::Label( "stop" ) );

        Matrix guard_constraints      = Matrix::Zero( 2, variableNames.size() );
        Vector guard_constants        = Vector::Zero( 2 );
        guard_constraints( 0, theta ) = 1;
        guard_constraints( 1, theta ) = -1;
        guard_constants << theta_max, -theta_min;

        stopTrans->setGuard( { guard_constraints, guard_constants } );
      } else if (differenceLeft == 0) {
        auto newThetaBucket = targetThetaBucket;
        auto thetaChange    = source->createTransition( buckets[std::make_tuple( segmentId, zone )] );
        thetaChange->addLabel( hypro::Label( "set_theta_" + std::to_string( newThetaBucket ) ) );

        Matrix guard_constraints  = Matrix::Zero( 2, variableNames.size() );
        Vector guard_constants    = Vector::Zero( 2 );
        guard_constraints( 0, theta ) = 1;
        guard_constraints( 1, theta ) = -1;
        guard_constants << theta_max, -theta_min;

        thetaChange->setGuard({guard_constraints, guard_constants});

      } else if (differenceLeft <= differenceRight) {
        auto turn = std::min(maxTurn, differenceLeft);
        auto newThetaBucket = t + turn >= theta_discretization ? t + turn - theta_discretization : t + turn;
        auto thetaChange    = source->createTransition( buckets[std::make_tuple( segmentId, zone )] );
        thetaChange->addLabel( hypro::Label( "set_theta_" + std::to_string( newThetaBucket ) ) );

        Matrix guard_constraints  = Matrix::Zero( 2, variableNames.size() );
        Vector guard_constants    = Vector::Zero( 2 );
        guard_constraints( 0, theta ) = 1;
        guard_constraints( 1, theta ) = -1;
        guard_constants << theta_max, -theta_min;

        thetaChange->setGuard({guard_constraints, guard_constants});

      } else if (differenceRight < differenceLeft) {
        auto turn = std::min(maxTurn, differenceRight);
        auto newThetaBucket = turn > t ? theta_discretization+t-turn : t - turn;
        auto thetaChange    = source->createTransition( buckets[std::make_tuple( segmentId, zone )] );
        thetaChange->addLabel( hypro::Label( "set_theta_" + std::to_string( newThetaBucket ) ) );

        Matrix guard_constraints  = Matrix::Zero( 2, variableNames.size() );
        Vector guard_constants    = Vector::Zero( 2 );
        guard_constraints( 0, theta ) = 1;
        guard_constraints( 1, theta ) = -1;
        guard_constants << theta_max, -theta_min;

        thetaChange->setGuard({guard_constraints, guard_constants});
      }

      theta_min += theta_increment;
      theta_max += theta_increment;
    }
  }


  // segment changes
  if ( numberOfSegments > 1 ) {
    for ( auto segmentId = 0; segmentId < numberOfSegments; segmentId++ ) {
      auto nextSegmentId     = segmentId < numberOfSegments - 1 ? segmentId + 1 : 0;
      auto previousSegmentId = segmentId > 0 ? segmentId - 1 : numberOfSegments - 1;

      for ( auto thisZoneId = 0; thisZoneId < 3; thisZoneId++ ) {
        for ( auto targetZoneId = 0; targetZoneId < 3; targetZoneId++ ) {
          auto thisLocation = buckets[std::tuple( segmentId, thisZoneId )];

          auto nextLocation = buckets[std::tuple( nextSegmentId, targetZoneId )];
          auto transNext    = thisLocation->createTransition( nextLocation );
          transNext->setGuard( nextLocation->getInvariant() );

          auto previousLocation = buckets[std::tuple( previousSegmentId, targetZoneId )];
          auto transPrevious    = thisLocation->createTransition( previousLocation );
          transPrevious->setGuard( previousLocation->getInvariant() );
        }
      }
    }
  }

  CarBaseController result{velocity, maxTurn, stopZoneWidth, borderAngle, theta_discretization};
  result.segments = segments;
  result.mAutomaton = res;

  return result;
}

}
