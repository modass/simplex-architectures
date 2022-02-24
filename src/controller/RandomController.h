//
// Created by bmaderbacher on 24.02.22.
//

#ifndef SIMPLEXARCHITECTURES_RANDOMCONTROLLER_H
#define SIMPLEXARCHITECTURES_RANDOMCONTROLLER_H

#include "AbstractController.h"
#include "../types.h"
#include <random>

namespace simplexArchitectures {

    class RandomController : public AbstractController<Point, Point> {
        Point generateInput(Point state) override;

        std::mt19937 generator;
        std::uniform_real_distribution<Number> dist = std::uniform_real_distribution<Number>( 0.0001, 0.0005 );
        std::discrete_distribution<int> disc_dist = std::discrete_distribution( { 1, 1 } );
    };

}

#endif //SIMPLEXARCHITECTURES_RANDOMCONTROLLER_H
