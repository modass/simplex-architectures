/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 21.02.22.
 */

#ifndef SIMPLEXARCHITECTURES_SAMPLINGUTILITY_H
#define SIMPLEXARCHITECTURES_SAMPLINGUTILITY_H

#include <hypro/util/adaptions_carl/adaptions_includes.h>
#include <hypro/datastructures/Point.h>

#include <vector>

namespace simplexArchitectures {

template <typename Number>
std::vector<carl::Interval<Number>> widenSample( const hypro::Point<Number>& sample, Number targetWidth,
                                                 std::vector<std::size_t> bloatingDimensions ) {
  std::vector<carl::Interval<Number>> intervals = std::vector<carl::Interval<Number>>( sample.dimension(), carl::Interval<Number>( 0 ) );
  Number                              range     = targetWidth / 2;
  for ( std::size_t i = 0; i < sample.dimension(); ++i ) {
    if ( std::find( bloatingDimensions.begin(), bloatingDimensions.end(), i ) != bloatingDimensions.end() ) {
      intervals[i] = carl::Interval<Number>( sample.at( i ) - range, sample.at( i ) + range );
    } else {
      intervals[i] = carl::Interval<Number>( sample.at( i ) );
    }
  }
  return intervals;
}

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_SAMPLINGUTILITY_H
