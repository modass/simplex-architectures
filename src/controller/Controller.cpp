/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 16.02.22.
 */

#include "Controller.h"

namespace simplexArchitectures {

hypro::Point<Number> Controller::operator()() {
  hypro::vector_t<double> coordinates = hypro::vector_t<double>::Zero( 1 );
  for ( std::size_t d = 0; d < 1; ++d ) {
    // value is with 50% probability zero and some value between 0 and 0.0005 otherwise
    //			coordinates( d ) = disc_dist( generator ) * dist( generator );
    coordinates( d ) = 0.0;
  }
  return hypro::Point<Number>{ coordinates };
}

hypro::Location<Number>* Controller::operator()( const hypro::HybridAutomaton<Number>& automaton ) {
  auto val = loc_dist( generator );
  return automaton.getLocations().at( val );
}

ControllerUpdate Controller::generateInput() {
  return { nullptr, this->operator()() };
}

}