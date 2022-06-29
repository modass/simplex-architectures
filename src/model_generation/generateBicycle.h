//
// Created by stefan on 22.03.22.
//

#ifndef SIMPLEXARCHITECTURES_GENERATEBICYCLE_H
#define SIMPLEXARCHITECTURES_GENERATEBICYCLE_H

#include <hypro/datastructures/HybridAutomaton/HybridAutomaton.h>
#include <hypro/datastructures/HybridAutomaton/output/Flowstar.h>

#include <string>

namespace modelGenerator {
/**
 * Computes a hybrid automaton based on a discretization of tan(\delta), cos(theta), sin(theta)
 * @param delta_ranges
 * @param discretization
 * @return
 */
hypro::HybridAutomaton<double> generateBicycle( std::pair<double, double> delta_ranges   = { -60, 60 },
                                                std::size_t delta_discretization = 7, std::size_t theta_discretization = 12 );

inline void generateBicycleModelFile( std::string filename, std::pair<double, double> delta_ranges = { -60, 60 },
                                      std::size_t delta_discretization = 7, std::size_t theta_discretization = 12 ) {
  std::ofstream fs{ filename };
  fs << hypro::toFlowstarFormat( generateBicycle( delta_ranges, delta_discretization, theta_discretization ) );
  fs.close();
}
}  // namespace modelGenerator

#endif  // SIMPLEXARCHITECTURES_GENERATEBICYCLE_H
