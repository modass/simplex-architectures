cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

project(simplexArchitectures CXX)
set(PROJECT_FULLNAME "SimplexArchitectures")
set(simplexArchitectures_VERSION "0.1")
set(PROJECT_VERSION "${simplexArchitectures_VERSION}")

# set caching of the build type to limit the choices for the user and to allow toggling
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug Release" FORCE)
endif ()

include(CMakePackageConfigHelpers)

# find dependencies
include(resources)

# invoke cmake for the subdirectories
add_subdirectory(src)
add_subdirectory(src/tool)
add_subdirectory(test)