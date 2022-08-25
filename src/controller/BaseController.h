/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 25.08.22.
 */

#ifndef SIMPLEXARCHITECTURES_BASECONTROLLER_H
#define SIMPLEXARCHITECTURES_BASECONTROLLER_H

#include "AbstractController.h"

namespace simplexArchitectures {

template<typename Automaton, typename State, typename Update>
struct BaseController : AbstractController<State,Update>{
  Automaton mAutomaton;
};

} // namespace

#endif  // SIMPLEXARCHITECTURES_BASECONTROLLER_H
