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

# make targets available
FetchContent_MakeAvailable(Catch2 cereal)