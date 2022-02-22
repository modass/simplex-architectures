/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 21.02.22.
 */

#ifndef SIMPLEXARCHITECTURES_TRAININGHEURISTICS_H
#define SIMPLEXARCHITECTURES_TRAININGHEURISTICS_H

#include <hypro/datastructures/HybridAutomaton/HybridAutomaton.h>

#include <random>

#include "../simulation/SamplingUtility.h"
#include "../types.h"
#include "TrainingSettings.h"

namespace simplexArchitectures {

using locationConditionMap = hypro::HybridAutomaton<Number>::locationConditionMap;

struct Random {
  locationConditionMap operator()( const hypro::HybridAutomaton<Number>& automaton, const TrainingSettings& settings ) {
    locationConditionMap                       res;
    std::mt19937                               generator;
    std::uniform_int_distribution<std::size_t> LocPtr_dist{ 0, automaton.getLocations().size() - 1 };
    std::size_t                                chosenLocPtr = LocPtr_dist( generator );
    LocPtr                                     loc = *std::next( automaton.getLocations().begin(), chosenLocPtr );

    // create sample
    Point sample{ hypro::vector_t<Number>::Zero( settings.samplingArea.size() ) };
    for ( auto i = 0; i < settings.samplingArea.size(); ++i ) {
      std::uniform_real_distribution<Number> sample_dist{ settings.samplingArea.interval( i ).lower(),
                                                          settings.samplingArea.interval( i ).upper() };
      sample[i] = sample_dist( generator );
    }
    // widen sample, if required
    res[loc] = hypro::Condition<Number>{ widenSample( sample, settings.initialSetWidth, settings.wideningDimensions ) };

    return res;
  }
};

struct Single {
  locationConditionMap operator()( const hypro::HybridAutomaton<Number>&, const TrainingSettings& settings ) {
    locationConditionMap res;
    res[loc] = hypro::Condition<Number>{ widenSample( sample, settings.initialSetWidth, settings.wideningDimensions ) };
    return res;
  }

  hypro::Point<Number>     sample;
  hypro::Location<Number>* loc;
};

struct Grid {
  Grid( const std::vector<std::size_t>& resolution, const TrainingSettings& settings )
      : resolution( resolution ),
        currentCell( std::vector<std::size_t>( 0, resolution.size() ) ),
        gen( resolution, settings.samplingArea.dimension() ) {
    if ( resolution.size() != settings.samplingArea.dimension() ) {
      throw std::logic_error( "Please specify a number of splittings for every dimension." );
    }
  }

  locationConditionMap operator()( const hypro::HybridAutomaton<Number>& automaton, const TrainingSettings& settings ) {
    locationConditionMap res;
    if ( gen.end() ) {
      gen.reset();
      locIndex++;
    }
    currentCell = gen();
    LocPtr loc  = *std::next( automaton.getLocations().begin(), locIndex );

    // create sample
    Point sample{ hypro::vector_t<Number>::Zero( settings.samplingArea.dimension() ) };
    for ( auto i = 0; i < settings.samplingArea.dimension(); ++i ) {
      const auto& interval = settings.samplingArea.at( i );
      sample[i]            = ( interval.upper() - interval.lower() ) / ( resolution[i] + 1 ) * ( currentCell[i] + 1 );
    }
    // widen sample, if required
    res[loc] = hypro::Condition<Number>{ widenSample( sample, settings.initialSetWidth, settings.wideningDimensions ) };

    return res;
  }

 private:
  std::vector<std::size_t> resolution;
  std::vector<std::size_t> currentCell;
  hypro::Combinator        gen;
  std::size_t              locIndex = 0;
};

struct GridCover {
  GridCover( const std::vector<std::size_t>& resolution, const TrainingSettings& settings )
      : resolution( resolution ),
        currentCell( std::vector<std::size_t>( 0, resolution.size() ) ),
        gen( resolution, settings.samplingArea.dimension() ) {
    if ( resolution.size() != settings.samplingArea.dimension() ) {
      throw std::logic_error( "Please specify a number of splittings for every dimension." );
    }
  }

  locationConditionMap operator()( const hypro::HybridAutomaton<Number>& automaton, const TrainingSettings& settings ) {
    locationConditionMap res;
    if ( gen.end() ) {
      gen.reset();
      locIndex++;
    }
    currentCell = gen();
    LocPtr loc  = *std::next( automaton.getLocations().begin(), locIndex );

    // create sample
    std::vector<carl::Interval<Number>> intervals;
    for ( auto i = 0; i < settings.samplingArea.dimension(); ++i ) {
      if ( resolution[i] == 1 ) {
        intervals.emplace_back( settings.samplingArea.at( i ) );
      } else {
        const auto& interval   = settings.samplingArea.at( i );
        Number      width      = ( interval.upper() - interval.lower() ) / resolution[i];
        Number      lowerBound = interval.lower() + currentCell[i] * width;
        Number      upperBound = interval.lower() + ( currentCell[i] + 1 ) * width;
        intervals.emplace_back( lowerBound, upperBound );
      }
    }
    assert( intervals.size() == settings.samplingArea.dimension() );
    // widen sample, if required
    res[loc] = hypro::conditionFromIntervals( intervals );

    return res;
  }

 private:
  std::vector<std::size_t> resolution;
  std::vector<std::size_t> currentCell;
  hypro::Combinator        gen;
  std::size_t              locIndex = 0;
};

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_TRAININGHEURISTICS_H
