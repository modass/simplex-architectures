//
// Created by bmaderbacher on 24.02.22.
//

#include "RandomController.h"

namespace simplexArchitectures {
    Point RandomController::generateInput(Point state) {
        hypro::vector_t<double> coordinates = hypro::vector_t<double>::Zero( 1 );
        // value is with 50% probability zero and some value between 0.0001 and 0.0005 otherwise
        coordinates( 0 ) = disc_dist( generator ) * dist( generator );
        return hypro::Point<Number>{ coordinates };
    }

}