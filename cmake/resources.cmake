# cmake module to make fetching of external ressources easier
include(FetchContent)
include(ExternalProject)

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

# triangle, a small tool to create 2D Delaunay triangulations
include(triangle)

# make targets available
FetchContent_MakeAvailable(Catch2 cereal cli11 spdlog)
list(APPEND CMAKE_MODULE_PATH ${Catch2_SOURCE_DIR}/contrib)
include(Catch)