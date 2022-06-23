
add_executable(tool_car tool.cpp ctrlConversion.cpp)
target_link_libraries(tool_car hypro-shared cereal::cereal SimplexArchitectures CLI11 model_generation)