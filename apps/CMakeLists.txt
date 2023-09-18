add_executable(basic_example examples/basic_example.cpp)

target_compile_features(basic_example PRIVATE cxx_std_17)

# Add libraries to the target...