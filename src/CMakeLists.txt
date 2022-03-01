add_library(SimplexArchitectures controller/Controller.cpp simulation/Simulator.cpp training/Trainer.cpp utility/octreePlotting.cpp)
target_include_directories(SimplexArchitectures PUBLIC ./)
target_link_libraries(SimplexArchitectures hypro-shared cereal::cereal spdlog)

set(INCLUDE_INSTALL_DIR include/ CACHE PATH "")
set(LIB_INSTALL_DIR lib/ CACHE PATH "")
set(SYSCONFIG_INSTALL_DIR etc/foo/ CACHE PATH "")

include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_SOURCE_DIR}/cmake/SimplexArchitecturesConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/SimplexArchitecturesConfig.cmake
        INSTALL_DESTINATION ${LIB_INSTALL_DIR}/SimplexArchitectures/cmake
        PATH_VARS INCLUDE_INSTALL_DIR SYSCONFIG_INSTALL_DIR)
write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/SimplexArchitecturesConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/SimplexArchitecturesConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/SimplexArchitecturesConfigVersion.cmake
        DESTINATION ${LIB_INSTALL_DIR}/SimplexArchitectures/cmake)
add_library(SimplexArchitectures controller/Controller.cpp simulation/Simulator.cpp training/Trainer.cpp controller/AbstractController.h controller/ConstantController.h controller/RandomController.cpp controller/RandomController.h controller/RLController.cpp controller/RLController.h)
target_link_libraries(SimplexArchitectures hypro-shared cereal::cereal)