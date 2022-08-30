
#pragma once
#include "ctrlConversion.h"

namespace simplexArchitectures {
template<typename Automaton>
std::vector<LocPtr> getLocationsForState(const Point& in, const Automaton& automaton) {
  std::vector<LocPtr> res;
  for(typename Automaton::LocationType* loc : automaton.getLocations()) {
    if(loc->getInvariant().contains(in)) {
      res.push_back(loc);
    }
  }
  return res;
}

} // namespace
