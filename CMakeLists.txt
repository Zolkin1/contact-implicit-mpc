cmake_minimum_required(VERSION 3.25)

project(hybrid-mpc VERSION 0.1
        DESCRIPTION "Model Predictive Control for Hybrid Systems"
        LANGUAGES CXX)

add_subdirectory(apps)

# Test executable to make sure everything is functioning
#add_executable(test tests/test1.cpp tests/test1.cpp)

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