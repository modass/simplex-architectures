
add_executable(tool tool.cpp)
target_link_libraries(tool hypro-shared cereal::cereal SimplexArchitectures CLI11)