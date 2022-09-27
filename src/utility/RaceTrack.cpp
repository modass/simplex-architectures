/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 25.03.22.
 */

#include "RaceTrack.h"

#include <hypro/util/plotting/Plotter.h>

namespace simplexArchitectures {

/*
std::vector<hypro::Condition<double>> RaceTrack::createSafetySpecification() const {
  assert(is_sane());
  std::vector<hypro::Condition<double>> res;
  // bloat-functor
  auto bloat = [this]( const auto& intv ) {
    return intv;
  };
  // add playground
  std::vector<carl::Interval<double>> intervals;
  const double borderWidth = 0.3;
  // left
  intervals.emplace_back( playground.intervals().at( 0 ).lower() - borderWidth,
                          playground.intervals().at( 0 ).lower() );
  intervals.emplace_back( playground.intervals().at( 1 ).lower(),
                          playground.intervals().at( 1 ).upper() + borderWidth );
  res.emplace_back( hypro::conditionFromIntervals( intervals ) );
  intervals.clear();
  // right
  intervals.emplace_back( playground.intervals().at( 0 ).upper(),
                          playground.intervals().at( 0 ).upper() + borderWidth);
  intervals.emplace_back( playground.intervals().at( 1 ).lower() - borderWidth,
                          playground.intervals().at( 1 ).upper() );
  res.emplace_back( hypro::conditionFromIntervals( intervals ) );
  intervals.clear();
  // bottom
  intervals.emplace_back( playground.intervals().at( 0 ).lower() - borderWidth,
                          playground.intervals().at( 0 ).upper() );
  intervals.emplace_back( playground.intervals().at( 1 ).lower() - borderWidth,
                          playground.intervals().at( 1 ).lower() );
  res.emplace_back( hypro::conditionFromIntervals( intervals ) );
  intervals.clear();
  // top
  intervals.emplace_back( playground.intervals().at( 0 ).lower(),
                          playground.intervals().at( 0 ).upper() + borderWidth );
  intervals.emplace_back( playground.intervals().at( 1 ).upper(),
                          playground.intervals().at( 1 ).upper() + borderWidth );
  res.emplace_back( hypro::conditionFromIntervals( intervals ) );
  intervals.clear();
  // all obstacles
    for ( const auto& obstacle : obstacles ) {
      std::vector<carl::Interval<double>> bloatedIntervals;
      std::transform( obstacle.intervals().begin(), obstacle.intervals().end(), std::back_inserter( bloatedIntervals ),
                      bloat );
      res.emplace_back( hypro::conditionFromIntervals( bloatedIntervals ) );
    }
  return res;
}
 */

void RaceTrack::addToPlotter( std::optional<Point> car, size_t color ) {
  assert(is_sane());
  if ( car.has_value() && ( car.value() ).dimension() != 3 ) {
    throw std::logic_error( "The tuple representing the car-position and heading is not 3-dimensional." );
  }
  hypro::Plotter<double>& plt = hypro::Plotter<double>::getInstance();
  // add playground and obstacles
  plt.addObject( playground.vertices() );
  std::for_each( std::begin( obstacles ), std::end( obstacles ),
                 [&plt]( const auto& obs ) { auto tmp = hypro::HPolytope<double>{obs.getMatrix(), obs.getVector()}; plt.addObject( tmp.vertices() ); } );
  // add waypoints, in order
//  plt.addOrderedObject( waypoints );
  // add safety specification
  auto redSettings = plt.settings();
  redSettings.fill = true;
  for ( const auto& specCondition : createSafetySpecification() ) {
    plt.addObject( hypro::HPolytope<Number>( specCondition.getMatrix(), specCondition.getVector() ).vertices(),
                   hypro::plotting::colors[color], redSettings );
  }
  // add start finish line
  double startFinishWidth = 1.0;
  plt.addObject({Point{startFinishX, startFinishYlow},
                       Point{startFinishX+startFinishWidth, startFinishYlow},
                       Point{startFinishX+startFinishWidth, startFinishYhigh},
                       Point{startFinishX, startFinishYhigh}},
                 hypro::plotting::colors[hypro::plotting::turquoise], redSettings);

  // add track segments
  auto segmentSettings = plt.settings();
  segmentSettings.fill = false;
  for(const auto& segment : roadSegments) {
    plt.addObject({ segment.startLeft, segment.startRight, segment.endLeft, segment.endRight},
                   hypro::plotting::colors[hypro::plotting::orange], segmentSettings);
    // add zone boundaries
//    plt.addPolyline({segment.getCenterStartLeft(0.2), segment.getCenterEndLeft(0.2)});
//    plt.addPolyline({segment.getCenterStartRight(0.2), segment.getCenterEndRight(0.2)});
  }


  // add car, if existing
  Point  carPosition{ car.value().at( 0 ), car.value().at( 1 ) };
  auto heading = Point{std::cos( car.value().at( 2 )), std::sin( car.value().at( 2 ))};
  auto offsetLeft = Point{std::cos( car.value().at( 2 ) + M_PI * 0.5), std::sin( car.value().at( 2 ) + M_PI * 0.5)};

  double car_size = 1.0;

  auto a = carPosition + car_size * heading;
  auto b = carPosition - car_size * heading + car_size * offsetLeft;
  auto c = carPosition - car_size * 0.4 * heading;
  auto d = carPosition - car_size * heading - car_size * offsetLeft;
  plt.addPolyline( { a, b, c, d, a } );
}

Number RaceTrack::getDistanceToBoundary( const Point& car ) {
  for ( const auto& segment : roadSegments ) {
    if ( segment.contains( car ) ) {
      // create halfspaces for the boundaries to compute the distance
      auto left  = hypro::Halfspace<Number>( std::vector<Point>{ segment.startLeft, segment.endLeft } );
      auto right = hypro::Halfspace<Number>( std::vector<Point>{ segment.startRight, segment.endRight } );
      // compute signed distance, take the absolute value since the constructor of the planes does not necessarily find
      // the correct normal vector direction
      auto dist_left  = std::abs( left.signedDistance( car.projectOn( { 0, 1 } ).rawCoordinates() ) );
      auto dist_right = std::abs( right.signedDistance( car.projectOn( { 0, 1 } ).rawCoordinates() ) );
      return std::min( dist_left, dist_right );
    }
  }
  // if we reach here, none of the segments contained the car, i.e., the car is not in a valid position
  throw std::logic_error( "The passed car is not contained in one of the track segments." );
}

}  // namespace simplexArchitectures