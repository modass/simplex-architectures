
add_executable(tool_car tool.cpp ctrlConversion.cpp)
target_link_libraries(tool_car hypro-shared cereal::cereal SimplexArchitectures CLI11 model_generation)

add_executable(car_simulator simulator.cpp ctrlConversion.cpp)
target_link_libraries(car_simulator hypro-shared cereal::cereal SimplexArchitectures CLI11 model_generation)

add_executable(car_simulator_simple simulator_simple.cpp ctrlConversion.cpp)
target_link_libraries(car_simulator_simple hypro-shared cereal::cereal SimplexArchitectures CLI11 model_generation)