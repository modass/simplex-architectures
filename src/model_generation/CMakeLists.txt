add_library(model_generation generateBicycle.cpp)
target_link_libraries(model_generation hypro-shared)
target_include_directories(model_generation PUBLIC ./)

add_executable(generator main.cpp)
target_link_libraries(generator model_generation spdlog CLI11)