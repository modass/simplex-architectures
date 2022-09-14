//
// Created by bmaderbacher on 02.09.22.
//

#ifndef SIMPLEXARCHITECTURES_RANDOMCARCONTROLLER_H
#define SIMPLEXARCHITECTURES_RANDOMCARCONTROLLER_H

#include "AbstractController.h"
#include "../types.h"
#include "../utility/RaceTrack.h"
#include "../utility/coordinate_util.h"
#include <random>

namespace simplexArchitectures {

class RandomCarController : public AbstractController<Point, Point> {
 public:
  RandomCarController( double v, size_t mT, size_t tD ) : velocity( v ), maxTurn( mT ), theta_discretization(tD) {
    distrib = std::uniform_int_distribution<int>(-int(mT), int(mT));
    gen = std::mt19937(rd());
  }

  Point generateInput( Point state );

  const double velocity = 1.0;
  const size_t maxTurn  = 1;
  const size_t theta_discretization;

  std::random_device rd;
  std::mt19937 gen;
  std::uniform_int_distribution<int> distrib;
};

}
#endif  // SIMPLEXARCHITECTURES_RANDOMCARCONTROLLER_H
