
add_executable(tool tool.cpp ../controller/Controller.cpp ../simulation/Simulator.cpp)
target_link_libraries(tool hypro-shared)