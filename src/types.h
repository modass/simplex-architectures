/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 16.02.22.
 */

#ifndef SIMPLEXARCHITECTURES_TYPES_H
#define SIMPLEXARCHITECTURES_TYPES_H

#include <hypro/algorithms/reachability/Reach.h>
#include <hypro/datastructures/HybridAutomaton/HybridAutomaton.h>
#include <hypro/datastructures/reachability/ReachTreev2.h>
#include <hypro/representations/GeometricObjectBase.h>

using Number               = double;
using Representation       = hypro::Box<Number>;
using Box                  = hypro::Box<Number>;
using LocPtr               = hypro::Location<Number> const*;
using Point                = hypro::Point<Number>;
using Automaton            = hypro::HybridAutomaton<Number>;
using ReachTreeNode        = hypro::ReachTreeNode<Representation, typename Automaton::LocationType>;
using ReachabilityAnalyzer = hypro::reachability::Reach<Representation, Automaton>;

#endif  // SIMPLEXARCHITECTURES_TYPES_H
