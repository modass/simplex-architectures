add_executable(TrainerTest TrainerTest.cpp)
target_link_libraries(TrainerTest PRIVATE Catch2::Catch2 cereal hypro-shared)
#target_include_directories(TrainerTest PRIVATE ${hypro_INCLUDE_DIRS})