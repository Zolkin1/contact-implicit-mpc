add_executable(basic_example basic_example.cpp)

target_compile_features(basic_example PRIVATE cxx_std_17)

#target_include_directories(basic_example PUBLIC ../include ../include/integration)

target_link_libraries(basic_example simulation)
