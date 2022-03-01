add_executable(TrainerTest TrainerTest.cpp)
target_link_libraries(TrainerTest PRIVATE Catch2::Catch2 SimplexArchitectures)
target_include_directories(TrainerTest
        PUBLIC $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/src>)

catch_discover_tests(TrainerTest)
add_executable(ControllerTest ControllerTest.cpp)
target_link_libraries(ControllerTest PRIVATE Catch2::Catch2 SimplexArchitectures)
target_include_directories(ControllerTest
        PUBLIC $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/src>)