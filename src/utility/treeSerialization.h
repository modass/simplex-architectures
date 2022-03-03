/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 03.03.22.
 */

#ifndef SIMPLEXARCHITECTURES_TREESERIALIZATION_H
#define SIMPLEXARCHITECTURES_TREESERIALIZATION_H

#include <string>
#include <map>
#include <hypro/datastructures/Hyperoctree.h>
#include "../types.h"

namespace simplexArchitectures {

std::map<std::string, hypro::Hyperoctree<Number>> loadTrees(std::string filename);

void saveTrees(std::string filename, const std::map<std::string, hypro::Hyperoctree<Number>>& trees);

} // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_TREESERIALIZATION_H
