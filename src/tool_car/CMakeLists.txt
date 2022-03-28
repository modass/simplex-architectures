
add_executable(tool_car tool.cpp)
target_link_libraries(tool_car hypro-shared cereal::cereal SimplexArchitectures CLI11 model_generation)