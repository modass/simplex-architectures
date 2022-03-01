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

# output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include(CMakePackageConfigHelpers)

# find dependencies
include(resources)

# invoke cmake for the subdirectories
add_subdirectory(src)
add_subdirectory(src/tool)
add_subdirectory(src/training)

# configure paths.h.in (helper file which allows to get the absolute path for model files)
configure_file(src/paths.h.in src/paths.h)

# setup for ctest
include(CTest)
add_subdirectory(test)