/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 23.02.22.
 */

#ifndef SIMPLEXARCHITECTURES_OCTREEPLOTTING_H
#define SIMPLEXARCHITECTURES_OCTREEPLOTTING_H

#include <hypro/datastructures/Hyperoctree.h>
#include <hypro/util/plotting/Plotter.h>

namespace simplexArchitectures {

/**
 * Adds the content of an octree to the passed plotter instance
 * @param octree The octree
 * @param plt The plotter instance
 * @param plotSets If true, the sets in uncovered nodes are also plotted
 */
void plotOctree( const hypro::Hyperoctree<double> &octree, hypro::Plotter<double> &plt, bool plotSets = false );

void plotOctrees( const std::map<std::string, hypro::Hyperoctree<double>>& octrees, hypro::Plotter<double>& plt, bool plotSets = false);

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_OCTREEPLOTTING_H
