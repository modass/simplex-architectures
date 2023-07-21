
#pragma once
#include <spdlog/fmt/bundled/ostream.h>
#include <spdlog/spdlog.h>

#include <regex>

#include "ctrlConversion.h"

namespace simplexArchitectures {
template <typename Automaton>
std::vector<typename Automaton::LocationType*> getLocationsForState( const Point& in, const Automaton& automaton ) {
  std::vector<typename Automaton::LocationType*> res;
  spdlog::trace("Get locations for point {}",in);
  for ( typename Automaton::LocationType* loc : automaton.getLocations() ) {
    if ( loc->getInvariant().contains( in ) ) {
      spdlog::trace("Point {} is contained in the invariant of location {}",in,loc->getName());
      res.push_back( loc );
    }
  }
  return res;
}

template <typename Location>
std::vector<Location*> getLocationForTheta( Number theta, std::size_t discretization,
                                            const std::vector<Location*>& in ) {
  std::vector<Location*> res;
  std::size_t            thetaBucket = getThetaBucket( theta, discretization );
  spdlog::trace( "Search for locations with theta bucket {} corresponding to theta = {}", thetaBucket, theta );
  std::regex theta_regex( ".*theta-" + std::to_string( thetaBucket ) + "((_.*)|$)" );
  for ( auto* l : in ) {
    if ( std::regex_match( l->getName(), theta_regex ) ) {
      res.push_back( l );
    }
  }
  return res;
}

}  // namespace simplexArchitectures
