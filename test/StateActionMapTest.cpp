//
// Created by bmaderbacher on 25.02.22.
//
#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>

#include "../src/utility/StateActionMap.h"


TEST_CASE( "Empty Map" ) {

    enum Actions{"a", "b", "c"};

    Point p = Point({0});
    Location loc;
    simplexArchitectures::StateActionMap<Box,Actions> saMap;

    REQUIRE(saMap.getAction(loc, p) == std::nullopt);
}
