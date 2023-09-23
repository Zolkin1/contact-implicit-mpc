project(simulation)

set(HEADER_LIST
        include/Simulator.h)

add_library(simulation
        src/Simulator.cpp
        ${HEADER_LIST})

target_include_directories(simulation PUBLIC include)

#target_include_directories(simulation PUBLIC ../ci_mpc/include)

target_compile_features(simulation PUBLIC cxx_std_17)

target_link_libraries(simulation drake::drake ci_mpc)