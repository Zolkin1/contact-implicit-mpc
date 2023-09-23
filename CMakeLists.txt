cmake_minimum_required(VERSION 3.25)

project(hybrid-mpc VERSION 0.1
        DESCRIPTION "Model Predictive Control for Hybrid Systems"
        LANGUAGES CXX)

set(CMAKE_PREFIX_PATH "$ENV{HOME}/drake")

find_package(drake CONFIG REQUIRED)

add_subdirectory(ci_mpc)

# TODO: Make this optional (i.e. only for testing, not needed for hardware)
add_subdirectory(simulation)

add_subdirectory(apps)

option(PACKAGE_TESTS "Build the tests" ON) #PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
if(PACKAGE_TESTS)
    enable_testing()
    include(FetchContent)
    FetchContent_Declare(
            googletest
            URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
    )
    FetchContent_MakeAvailable(googletest)
    add_subdirectory(tests)
endif()