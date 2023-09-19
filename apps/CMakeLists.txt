add_executable(basic_example basic_example.cpp)

target_compile_features(basic_example PRIVATE cxx_std_17)

target_link_libraries(basic_example PUBLIC ci_mpc)

# Add libraries to the target...