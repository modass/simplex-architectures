set(JUST_INSTALL_CEREAL ON)
set(SKIP_PORTABILITY_TEST ON)
set(BUILD_TESTS OFF)

FetchContent_Declare(
        cereal
        GIT_REPOSITORY https://github.com/USCiLab/cereal.git
        GIT_TAG v1.3.1
)