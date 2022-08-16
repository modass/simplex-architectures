/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 21.02.22.
 */

#ifndef SIMPLEXARCHITECTURES_REACHTREEUTILITY_H
#define SIMPLEXARCHITECTURES_REACHTREEUTILITY_H

#include <hypro/datastructures/reachability/ReachTreev2.h>
#include <hypro/datastructures/reachability/ReachTreev2Util.h>

#include <vector>

#include "../types.h"

namespace simplexArchitectures {

bool hasFixedPoint( const std::vector<ReachTreeNode> &roots );

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_REACHTREEUTILITY_H
