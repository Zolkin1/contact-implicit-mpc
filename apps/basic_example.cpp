//
// Created by zach on 9/18/23.
// Copyright (c) 2023 Zachary Olkin. All rights reserved.
//


#include <iostream>
#include <thread>

#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/parsers/utils.hpp"

#include "SystemModel.h"
#include "SymplecticEuler.h"
#include "Simulator.h"

constexpr bool IS_FLOATING_BASE = true;

int main() {
    pinocchio::Model model;
    pinocchio::buildModels::manipulator(model);

    // ----- Arm ----- //
//    const std::string fr3_urdf_path = "../../apps/examples/arm_models/fr3_algr.urdf";
//    const std::string fr3_package_name = "models";
//    const std::string fr3_package_path = "../../apps/examples/arm_models/";
//    cimpc::SystemModel robot(fr3_urdf_path, IS_FLOATING_BASE);
//    robot.SetMeshPackageInformation(fr3_package_name, fr3_package_path);

    // ----- Mini Cheetah ----- //
    // Mini cheetah has:
    // 12 real joint (pinocchio lists 14, but one has no configuration space and the other is the floating base)
    // 12 joint configs and 12 joint velocities
    // 1 free floating base (represented as a position R^3 and a quaternion R^4) for a 7 dof base
    // This should total 12+12+6+6 = 36 DoF system (not using a quaternion) or 12+12+7+6 = 37 DoF system (with quaternion)
    // There are also 3 directions of contact forces per leg for an additional 12 forces
    const std::string mini_cheetah_urdf_path = "../../apps/examples/mini_cheetah/mini_cheetah_mesh.urdf";
    cimpc::SystemModel robot(mini_cheetah_urdf_path, IS_FLOATING_BASE);

    robot.PrintSystemInfo();

    // ---- Create the Integrator ------ //
    cimpc::integration::SymplecticEuler integrator(0.01);

    // ----- Create the simulator/visualizer ----- //
    cimpc::simulator::Simulator simulator(robot);
    std::cout << "Number of states: " << simulator.GetNumStates() << std::endl;
    std::cout << "Number of continuous states: " << simulator.GetNumContinuousStates() << std::endl;

    // ----- Integrate the state with pinocchio dynamics ----- //
    // TODO: Maybe make the state a class so I can access its elements easier (i.e. via a .x or .qw etc.. function)
    cimpc::JointVector initial_condition = robot.GetPose();
    initial_condition[2] = 3;   // set the z height so it can fall

    simulator.StartRecording(1);
    robot.SetPose(initial_condition);
    simulator.SetState(robot.GetPose());

    double Tf = 0.5;
    integrator.Advance(robot, Tf);
    robot.PrintState();
    std::cout << std::endl;
    simulator.SetState(robot.GetPose(), Tf);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    integrator.Advance(robot, Tf);
    robot.PrintState();
    std::cout << std::endl;
    simulator.SetState(robot.GetPose(), Tf+Tf);
    //std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    simulator.StopRecording();

    std::cin.get();

    return 0;
}
