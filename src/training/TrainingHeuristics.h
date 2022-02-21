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

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_TRAININGHEURISTICS_H
