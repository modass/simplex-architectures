/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 16.02.22.
 */

#ifndef SIMPLEXARCHITECTURES_CONTROLLER_H
#define SIMPLEXARCHITECTURES_CONTROLLER_H

#include "../types.h"
#include "ControllerUpdate.h"

#include <random>

namespace simplexArchitectures {

struct Controller {

hypro::Point<Number> operator()();

hypro::Location<Number>* operator()( const hypro::HybridAutomaton<Number>& automaton );

ControllerUpdate generateInput();

std::mt19937 generator;
std::uniform_int_distribution<int> loc_dist{ 0, 23 };
// std::uniform_real_distribution<Number> dist = std::uniform_real_distribution<Number>( 0, 0.0005 );
std::uniform_real_distribution<Number> dist = std::uniform_real_distribution<Number>( 0.0003, 0.0005 );
std::discrete_distribution<int> disc_dist = std::discrete_distribution( { 10, 90 } );
};

}
#endif // SIMPLEXARCHITECTURES_CONTROLLER_H
