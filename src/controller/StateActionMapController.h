//
// Created by bmaderbacher on 07.09.22.
//

#ifndef SIMPLEXARCHITECTURES_STATEACTIONMAPCONTROLLER_H
#define SIMPLEXARCHITECTURES_STATEACTIONMAPCONTROLLER_H

#include "../types.h"
#include "../utility/StateActionMap.h"
#include "AbstractController.h"
#include "../tool_car/ctrlConversion.h"

#include <utility>

namespace simplexArchitectures {

class StateActionMapController : public AbstractController<Point, Point> {
 public:
  StateActionMapController( double v, size_t tD, StateActionMap<Box,hypro::Label>& sam, Automaton& atm )
      : velocity( v ), theta_discretization( tD ), stateActionMap(sam), automaton(atm) {}

  Point generateInput( Point state ) override;

  double                   velocity = 1.0;
  std::size_t              theta_discretization;
  StateActionMap<Box,hypro::Label>& stateActionMap;
  Automaton&                automaton;

};

}  // namespace simplexArchitectures

#include "CarBaseController.tpp"

#endif  // SIMPLEXARCHITECTURES_STATEACTIONMAPCONTROLLER_H
