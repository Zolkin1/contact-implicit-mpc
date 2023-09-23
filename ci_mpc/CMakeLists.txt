project(ci_mpc)

set(HEADER_LIST include/integration/Integrator.h
        include/SystemModel.h
        include/integration/SymplecticEuler.h
        include/Controller.h
        include/PIDController.h)

add_library(ci_mpc
        src/integration/Integrator.cpp
        src/SystemModel.cpp
        src/integration/SymplecticEuler.cpp
        src/Controller.cpp
        src/PIDController.cpp
        ${HEADER_LIST})

target_include_directories(ci_mpc PUBLIC include/integration include)

target_compile_features(ci_mpc PUBLIC cxx_std_17)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pinocchio)

target_link_libraries(ci_mpc Eigen3::Eigen pinocchio)
