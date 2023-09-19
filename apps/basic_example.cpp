//
// Created by zach on 9/18/23.
// Copyright (c) 2023 Zachary Olkin. All rights reserved.
//

#include <iostream>

#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/container/aligned-vector.hpp"

#include "SystemModel.h"

int main() {

    std::cout << "Hello World from the examples!" << std::endl;

    //cimpc::SymplecticEuler<double> integrator;

    pinocchio::Model model;
    pinocchio::buildModels::manipulator(model);

    cimpc::SystemModel<double> robot;


    auto ddq = robot.GetForwardDynamics();

    std::cout << ddq.data() << std::endl;

    return 0;
}
