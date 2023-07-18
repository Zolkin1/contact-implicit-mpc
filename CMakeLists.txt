cmake_minimum_required(VERSION 3.25)

project(hybrid-mpc VERSION 0.1
        DESCRIPTION "Model Predictive Control for Hybrid Systems"
        LANGUAGES CXX)

# Test executable to make sure everything is functioning
add_executable(test tests/test1.cpp tests/test1.cpp)

