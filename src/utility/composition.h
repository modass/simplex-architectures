/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 02.09.22.
 */

#ifndef SIMPLEXARCHITECTURES_COMPOSITION_H
#define SIMPLEXARCHITECTURES_COMPOSITION_H

namespace simplexArchitectures {

template <typename Composed, typename Automaton>
Composed compose( const Automaton& carModel, const Automaton& bcAtm,
                  std::map<std::string, std::vector<hypro::Location<Number>*>> variableMap, reduceAutomaton )

}

#endif  // SIMPLEXARCHITECTURES_COMPOSITION_H
