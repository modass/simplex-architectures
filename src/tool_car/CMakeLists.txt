add_executable(tool_car tool.cpp ctrlConversion.cpp)
target_link_libraries(tool_car hypro-shared cereal::cereal SimplexArchitectures CLI11 model_generation)

add_dependencies(tool_car ressources)

add_executable(tool_car_repair repair_tool.cpp ctrlConversion.cpp)
target_link_libraries(tool_car_repair hypro-shared cereal::cereal SimplexArchitectures CLI11 model_generation)