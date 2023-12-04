
add_executable(tool tool.cpp)
target_link_libraries(tool hypro-shared hypro-parser cereal::cereal SimplexArchitectures CLI11)