//
// Created by zach on 9/18/23.
// Copyright (c) 2023 Zachary Olkin. All rights reserved.
//

#include <iostream>

#include "SystemModel.h"

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

namespace cimpc {

    template <typename scalar>
    SystemModel<scalar>::SystemModel() {
        std::cout << "in the default robot model constructor" << std::endl;
    }

    template <typename scalar>
    SystemModel<scalar>::SystemModel(const pinocchio::Model& model) : model_(model), controller_(nullptr) {
        std::cout << "in the robot model constructor" << std::endl;

        joint_config_ = pinocchio::neutral(model_);
        joint_velocity_ = Eigen::Matrix<scalar, Eigen::Dynamic, 1>::Zero(model_.nv);
    }

    template <typename scalar>
    SystemModel<scalar>::SystemModel(const pinocchio::Model& model, std::unique_ptr<Controller<scalar>> controller) :
                        model_(model), controller_(std::move(controller)) {
        joint_config_ = pinocchio::neutral(model_);
        joint_velocity_ = Eigen::Matrix<scalar, Eigen::Dynamic, 1>::Zero(model_.nv);
    }

    template <typename scalar>
    JointVector SystemModel<scalar>::GetForwardDynamics() {
        // TODO: Should move data generation elsewhere/need to see how expensive it is here
        pinocchio::Data data(model_);
        InputVector tau = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(joint_config_.rows());

        if (controller_ != nullptr) {
            tau = controller_->GetControl(joint_config_, joint_velocity_);
        }

        auto external_force = GetContactForces();

        JointVector ddq = pinocchio::aba(model_, data, joint_config_, joint_velocity_, tau, external_force);

        std::cout << "in the dynamics function" << std::endl;

        return ddq;
    }

    // TODO: Write
    template <typename scalar>
    pinocchio::container::aligned_vector<pinocchio::ForceTpl<scalar>> SystemModel<scalar>::GetContactForces() {
        // Given the current joint configuration determine how far I am from a contact

        // Given the distance from a contact, calculate the force

        PINOCCHIO_ALIGNED_STD_VECTOR(pinocchio::Force) force;

        return force;
    }

    // TODO: keep this here or put it all in a .h?
    template class SystemModel<double>;
}