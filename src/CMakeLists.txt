add_library(SimplexArchitectures controller/Controller.cpp simulation/Simulator.cpp training/Trainer.cpp)
target_link_libraries(SimplexArchitectures hypro-shared cereal::cereal)