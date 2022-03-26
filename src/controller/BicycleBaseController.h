/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 25.03.22.
 */

#ifndef SIMPLEXARCHITECTURES_BICYCLEBASECONTROLLER_H
#define SIMPLEXARCHITECTURES_BICYCLEBASECONTROLLER_H

#include "AbstractController.h"
#include "../types.h"

namespace simplexArchitectures {

class BicycleBaseController : public AbstractController<Point, Point> {

  Point generateInput(Point state);
};

}

#endif  // SIMPLEXARCHITECTURES_BICYCLEBASECONTROLLER_H
