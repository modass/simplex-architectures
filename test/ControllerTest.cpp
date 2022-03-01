//
// Created by bmaderbacher on 25.02.22.
//
#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>

#include "../src/controller/AbstractController.h"
#include "../src/controller/ConstantController.h"
#include "../src/controller/RandomController.h"
#include "../src/controller/RLController.h"
#include "../src/types.h"
#include "paths.h"


TEST_CASE( "Test Execute ConstantController" ) {

    Point output = Point({0});
    simplexArchitectures::ConstantController<Point,Point> c = simplexArchitectures::ConstantController<Point,Point>(output);
    simplexArchitectures::AbstractController<Point,Point> &ctrl = c;

    Point o1 = ctrl.generateInput(Point({0.1,0.1}));
    REQUIRE(o1.at(0) == 0);

    Point o2 = ctrl.generateInput(Point({0.1,0.2}));
    REQUIRE(o2.at(0) == 0);

    Point o3 = ctrl.generateInput(Point({0.3,0.5}));
    REQUIRE(o3.at(0) == 0);
}


TEST_CASE( "Test Execute RandomController" ) {

    simplexArchitectures::RandomController c = simplexArchitectures::RandomController();
    simplexArchitectures::AbstractController<Point,Point> &ctrl = c;

    Point o1 = ctrl.generateInput(Point({0.1,0.1}));
    REQUIRE(o1.at(0) >= 0);
    REQUIRE(o1.at(0) <= 0.0005);

    Point o2 = ctrl.generateInput(Point({0.1,0.2}));
    REQUIRE(o2.at(0) >= 0);
    REQUIRE(o2.at(0) <= 0.0005);

    Point o3 = ctrl.generateInput(Point({0.3,0.5}));
    REQUIRE(o3.at(0) >= 0);
    REQUIRE(o3.at(0) <= 0.0005);
}


TEST_CASE( "Test Execute RLController" ) {

    simplexArchitectures::RLController c = simplexArchitectures::RLController(simplexArchitectures::getModelsPath()+"../networks/watertanks.txt");
    simplexArchitectures::AbstractController<Point,Point> &ctrl = c;

    Point o1 = ctrl.generateInput(Point({0.1,0.1}));
    REQUIRE(o1.at(0) >= 0);
    REQUIRE(o1.at(0) <= 0.0005);

    Point o2 = ctrl.generateInput(Point({0.1,0.2}));
    REQUIRE(o2.at(0) >= 0);
    REQUIRE(o2.at(0) <= 0.0005);

    Point o3 = ctrl.generateInput(Point({0.3,0.5}));
    REQUIRE(o3.at(0) >= 0);
    REQUIRE(o3.at(0) <= 0.0005);
}