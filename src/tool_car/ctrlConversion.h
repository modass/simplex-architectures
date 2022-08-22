/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 23.06.22.
 */

#ifndef SIMPLEXARCHITECTURES_CTRLCONVERSION_H
#define SIMPLEXARCHITECTURES_CTRLCONVERSION_H

#include "../types.h"
#include <hypro/datastructures/HybridAutomaton/HybridAutomaton.h>

namespace simplexArchitectures {

/**
 * Converts the output of the controller into a location of the autmaton.
 * @param in The returned control output from the controller
 * @param automaton The discretized plant
 * @param lastLocation The last location control was in, required to chose the correct theta-bucket
 * @param discretization The number of buckets
 * @param delta_ranges The range of delta, which is discretized
 * @return A pointer to the location in the plant that corresponds to the dynamics resulting from the given ctlr output
 */
LocPtr convertCtrlToLocation(const Point& in, const hypro::HybridAutomaton<Number>& automaton, LocPtr lastLocation, std::size_t discretization, const std::pair<double, double>& delta_ranges);

LocPtr convertCtrlToLocationSimple(double theta, const hypro::HybridAutomaton<Number>& automaton, std::size_t theta_discretization);

double convertDeltaToTheta(double delta, double currentTheta, std::size_t theta_discretization);

std::size_t getThetaBucket(Number theta, std::size_t discretization);

std::size_t getDeltaBucket(Number delta, const std::pair<double,double>& delta_ranges, std::size_t discretization);

std::size_t getXBucket(Number x, double x_min, double x_max, double x_interval_size);

std::size_t getYBucket(Number y, double y_min, double y_max, double y_interval_size);

template<typename Automaton>
std::vector<LocPtr> getLocationsForState(const Point& in, const Automaton& automaton);

std::vector<LocPtr> getLocationForTheta(Number theta, std::size_t discretization, const std::vector<LocPtr>& in);

} // namespace

#include "ctrlConversion.tpp"

#endif  // SIMPLEXARCHITECTURES_CTRLCONVERSION_H
