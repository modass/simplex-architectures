/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 20.02.22.
 */
#define CATCH_CONFIG_MAIN

#include "../src/training/Trainer.h"
#include <catch2/catch.hpp>

TEST_CASE("Construct Trainer") { simplexArchitectures::Trainer t{"testfile"}; }
