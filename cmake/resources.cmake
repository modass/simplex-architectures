# find hypro
find_package(hypro CONFIG REQUIRED)
if(${hypro_FOUND})
    message(STATUS "HyPro found: ${hypro_LIBRARIES}")
endif()