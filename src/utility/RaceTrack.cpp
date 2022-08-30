/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 25.03.22.
 */

#include "RaceTrack.h"

#include <hypro/util/plotting/Plotter.h>

namespace simplexArchitectures {

std::vector<hypro::Condition<double>> RaceTrack::createSafetySpecification() const {
  assert(is_sane());
  std::vector<hypro::Condition<double>> res;
  // bloat-functor
  auto bloat = [this]( const auto& intv ) {
    return carl::Interval<double>( intv.lower() - safetyMargin, intv.upper() + safetyMargin );
  };
  // add playground
  std::vector<carl::Interval<double>> intervals;
  // left
  intervals.emplace_back( playground.intervals().at( 0 ).lower(),
                          playground.intervals().at( 0 ).lower() + safetyMargin );
  intervals.emplace_back( playground.intervals().at( 1 ) );
  res.emplace_back( hypro::conditionFromIntervals( intervals ) );
  intervals.clear();
  // right
  intervals.emplace_back( playground.intervals().at( 0 ).upper() - safetyMargin,
                          playground.intervals().at( 0 ).upper() );
  intervals.emplace_back( playground.intervals().at( 1 ) );
  res.emplace_back( hypro::conditionFromIntervals( intervals ) );
  intervals.clear();
  // bottom
  intervals.emplace_back( playground.intervals().at( 0 ) );
  intervals.emplace_back( playground.intervals().at( 1 ).lower(),
                          playground.intervals().at( 1 ).lower() + safetyMargin );
  res.emplace_back( hypro::conditionFromIntervals( intervals ) );
  intervals.clear();
  // top
  intervals.emplace_back( playground.intervals().at( 0 ) );
  intervals.emplace_back( playground.intervals().at( 1 ).upper() - safetyMargin,
                          playground.intervals().at( 1 ).upper() );
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

void RaceTrack::addToPlotter( std::optional<Point> car, size_t color ) {
  assert(is_sane());
  if ( car.has_value() && ( car.value() ).dimension() != 3 ) {
    throw std::logic_error( "The tuple representing the car-position and heading is not 3-dimensional." );
  }
  hypro::Plotter<double>& plt = hypro::Plotter<double>::getInstance();
  // add playground and obstacles
  plt.addObject( playground.vertices() );
  std::for_each( std::begin( obstacles ), std::end( obstacles ),
                 [&plt]( const auto& obs ) { plt.addObject( obs.vertices() ); } );
  // add waypoints, in order
  plt.addOrderedObject( waypoints );
  // add safety specification
  auto redSettings = plt.settings();
  redSettings.fill = true;
  for ( const auto& specCondition : createSafetySpecification() ) {
    plt.addObject( hypro::Box<double>( specCondition.getMatrix(), specCondition.getVector() ).vertices(),
                   hypro::plotting::colors[color], redSettings );
  }
  // add car, if existing
  Point  carPosition{ car.value().at( 0 ), car.value().at( 1 ) };
  double xArrowOffset = std::cos( car.value().at( 2 ));
  double yArrowOffset = std::sin( car.value().at( 2 ));
  Point  carDirection{ xArrowOffset, yArrowOffset };
  std::cout << "Direction: " << carDirection << std::endl;
  plt.addVector( ( carPosition + carDirection ).rawCoordinates(), carPosition.rawCoordinates() );
  plt.addPoint( carPosition );
}

}  // namespace simplexArchitectures