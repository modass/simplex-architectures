//
// Created by bmaderbacher on 06.07.22.
//

#include "generateSimpleBaseController.h"

#include "../tool_car/ctrlConversion.h"
#include "../utility/coordinate_util.h"
#include "spdlog/spdlog.h"

namespace simplexArchitectures {

BicycleBaseController generateSimpleBaseController(std::size_t theta_discretization, size_t maxTurn, //in theta buckets
                                                                                   double stopZoneWidth,
                                                                                   double borderAngle,
                                                                                   const std::vector<RoadSegment>& segments,
                                                    double velocity) {

  using Matrix = hypro::matrix_t<double>;
  using Vector = hypro::vector_t<double>;

  hypro::HybridAutomaton<double> res;
  constexpr Eigen::Index         x     = 0;  // global position x
  constexpr Eigen::Index         y     = 1;  // global position y
  constexpr Eigen::Index         theta = 2;  // global position y
  constexpr Eigen::Index         C     = 3;  // constants

  std::vector<std::string> variableNames{ "x", "y", "theta" };
  res.setVariables( variableNames );

  auto numberOfSegments = segments.size();

  // sync labels: theta_{0..theta_discretization-1}, stop

  // zones: borderLeft, stop, borderRight
  // bucket indices: (segment, zone)
  std::map<std::tuple<std::size_t, std::size_t>, hypro::Location<double>*>               buckets;
  std::map<std::tuple<std::size_t, std::size_t, std::size_t>, std::pair<double, double>> outputs;

  for ( std::size_t is = 0; is < numberOfSegments; ++is ) {
    auto segment    = segments[is];
    bool horizontal = segment.orientation == LeftToRight || segment.orientation == RightToLeft;

    // TODO add assertions to check that the arithmetic used to calculate the boundaries produces valid intervals

    for ( std::size_t iz = 0; iz < 3; ++iz ) {
      double x_low;
      double y_low;
      double x_high;
      double y_high;

      if ( horizontal ) {
        x_low    = segment.x_min;
        x_high   = segment.x_max;
        auto mid = segment.y_min + ( segment.y_max - segment.y_min ) / 2.0;
        if ( iz == 1 ) {
          y_low  = mid - stopZoneWidth / 2;
          y_high = mid + stopZoneWidth / 2;
        } else if ( ( iz == 0 && segment.orientation == LeftToRight ) ||
                    ( iz == 2 && segment.orientation == RightToLeft ) ) {
          y_low  = mid + stopZoneWidth / 2;
          y_high = segment.y_max;
        } else if ( ( iz == 0 && segment.orientation == RightToLeft ) ||
                    ( iz == 2 && segment.orientation == LeftToRight ) ) {
          y_high = mid - stopZoneWidth / 2;
          y_low  = segment.y_min;
        }
      } else {  // vertical
        y_low    = segment.y_min;
        y_high   = segment.y_max;
        auto mid = segment.x_min + ( segment.x_max - segment.x_min ) / 2.0;
        if ( iz == 1 ) {
          x_low  = mid - stopZoneWidth / 2;
          x_high = mid + stopZoneWidth / 2;
        } else if ( ( iz == 2 && segment.orientation == BottomToTop ) ||
                    ( iz == 0 && segment.orientation == TopToBottom ) ) {
          x_low  = mid + stopZoneWidth / 2;
          x_high = segment.x_max;
        } else if ( ( iz == 2 && segment.orientation == TopToBottom ) ||
                    ( iz == 0 && segment.orientation == BottomToTop ) ) {
          x_high = mid - stopZoneWidth / 2;
          x_low  = segment.x_min;
        }
      }

      auto loc = res.createLocation();
      buckets.emplace( std::make_tuple( is, iz ), loc );
      loc->setName( "segment_" + std::to_string( is ) + "_zone_" + std::to_string( iz ) );

      // compute and set invariants
      // ATTENTION: The order of constraints is important, it is reused for guards later!!!
      Matrix invariant_constraints  = Matrix::Zero( 4, variableNames.size() );
      Vector invariant_constants    = Vector::Zero( 4 );
      invariant_constraints( 0, x ) = 1;
      invariant_constraints( 1, x ) = -1;
      invariant_constraints( 2, y ) = 1;
      invariant_constraints( 3, y ) = -1;
      invariant_constants << x_high, -x_low, y_high, -y_low;
      loc->setInvariant( { invariant_constraints, invariant_constants } );
    }
  }

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
    double segmentAngle;
    switch ( segment.orientation ) {
      case LeftToRight:
        segmentAngle = 0.0;
        break;
      case RightToLeft:
        segmentAngle = M_PI;
        break;
      case BottomToTop:
        segmentAngle = 0.5 * M_PI;
        break;
      case TopToBottom:
        segmentAngle = 1.5 * M_PI;
        break;
    }

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

  BicycleBaseController result;
  result.segments = segments;
  result.borderAngle = borderAngle;
  result.stopZoneWidth = stopZoneWidth;
  result.theta_discretization = theta_discretization;
  result.maxTurn = maxTurn;
  result.mAutomaton = res;
  result.velocity = velocity;

  return result;
}

} // namespace