# cmake module to make fetching of external ressources easier
include(FetchContent)

# find hypro
find_package(hypro CONFIG REQUIRED)
if (${hypro_FOUND})
    message(STATUS "HyPro found: ${hypro_LIBRARIES}")
endif ()

# obtain cereal, a library for serialization of datastructures
include(cereal)

# add catch2, a unittesting framework
include(catch2)

# a cli-parser
include(cli11)

# logging framework
include(spdlog)

# make targets available
FetchContent_MakeAvailable(Catch2 cereal cli11 spdlog)
list(APPEND CMAKE_MODULE_PATH ${Catch2_SOURCE_DIR}/contrib)
include(Catch)