//
// Created by stefan on 22.03.22.
//

#ifndef SIMPLEXARCHITECTURES_GENERATECARMODEL_H
#define SIMPLEXARCHITECTURES_GENERATECARMODEL_H

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
hypro::HybridAutomaton<double> generateCarModel(std::size_t theta_discretization = 12, double cycleTime = 0.1, double bcvelocity = 1.0, bool includeThetaTransitions = true);

inline void generateCarModelFile( std::string filename, std::size_t theta_discretization = 12, double cycleTime = 0.1, double bcvelocity = 1.0, bool includeThetaTransitions = true ) {
  std::ofstream fs{ filename };
  fs << hypro::toFlowstarFormat( generateCarModel( theta_discretization, cycleTime, bcvelocity, includeThetaTransitions ) );
  fs.close();
}
}  // namespace modelGenerator

#endif  // SIMPLEXARCHITECTURES_GENERATECARMODEL_H
