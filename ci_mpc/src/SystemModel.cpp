//
// Created by zach on 9/18/23.
// Copyright (c) 2023 Zachary Olkin. All rights reserved.
//

#include <iostream>

#include "SystemModel.h"

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace cimpc {

    SystemModel::SystemModel() {
        std::cout << "in the default robot model constructor" << std::endl;
    }

    SystemModel::SystemModel(const pinocchio::Model& model,
                                     std::unique_ptr<Controller> controller) :
                        model_(model), controller_(std::move(controller)) {
        joint_config_ = pinocchio::neutral(model_);
        joint_velocity_ = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(model_.nv);

        model_data_ = std::make_unique<pinocchio::Data>(model_);
    }

    SystemModel::SystemModel(const std::string& file_path, const bool& is_floating_base,
                             std::unique_ptr<Controller> controller) : controller_(std::move(controller)) {
        is_floating_base_ = is_floating_base;
        if (is_floating_base_) {
            pinocchio::urdf::buildModel(file_path, pinocchio::JointModelFreeFlyer(), model_, true);
        }
        else {
            pinocchio::urdf::buildModel(file_path, model_, true);
        }

        urdf_file_path_ = file_path;

        joint_config_ = pinocchio::neutral(model_);
        joint_velocity_ = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(model_.nv);
        model_data_ = std::make_unique<pinocchio::Data>(model_);
    }

    JointVector SystemModel::GetForwardDynamics() {
        InputVector tau = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(model_.nv);
        //tau[0] = 1;

        if (controller_ != nullptr) {
            tau = controller_->GetControl(joint_config_, joint_velocity_);
        }

        auto external_force = GetContactForces();

        // I believe aba is the most efficient way to get the forward dynamics if I don't need
        // any of the other terms. If I need the other terms I can call forwardDynamics();
        JointVector ddq = pinocchio::aba(model_, *model_data_, joint_config_, joint_velocity_, tau); //, external_force);

        return ddq;
    }

    // TODO: Write
    pinocchio::container::aligned_vector<pinocchio::ForceTpl<double>> SystemModel::GetContactForces() {
        // Given the current joint configuration determine how far I am from a contact

        // Given the distance from a contact, calculate the force

        PINOCCHIO_ALIGNED_STD_VECTOR(pinocchio::Force) force(1);

        return force;
    }

    JointVector SystemModel::GetPose() {
        return joint_config_;
    }

    JointVector SystemModel::GetJointVelocity() {
        return joint_velocity_;
    }

    void SystemModel::SetMeshPackageInformation(std::string mesh_package_name, std::string mesh_package_path) {
        mesh_package_name_ = mesh_package_name;
        mesh_package_path_ = mesh_package_path;
    }

    void SystemModel::SetPose(const JointVector& joint_config) {
        joint_config_ = joint_config;
    }

    void SystemModel::SetJointVelocity(const JointVector& joint_velocity) {
        joint_velocity_ = joint_velocity;
    }

    void SystemModel::PrintState() const {
        std::cout << "Joint Configuration: " << joint_config_.transpose() << std::endl;
        std::cout << "Joint Velocities: " << joint_velocity_.transpose() << std::endl;
    }

    std::string SystemModel::GetURDFFilePath() const {
        return urdf_file_path_;
    }

    int SystemModel::GetConfigurationDimension() const {
        return model_.nq;
    }

    int SystemModel::GetVelocityDimension() const {
        return model_.nv;
    }

    void SystemModel::PrintSystemInfo() const {
        std::cout << "Model name: " << model_.name << std::endl;
        std::cout << "Number of  bodies: " << model_.nbodies << std::endl;
        std::cout << "Configuration space dimension: " << GetConfigurationDimension() << std::endl;
        std::cout << "Velocity space dimension: " << GetVelocityDimension() << std::endl;
        std::cout << "Number of joints: " << model_.njoints << std::endl;

        std::cout << "Joint specific configuration space dimensions: ";
        std::vector<int> joint_config_spaces = model_.nqs;
        auto iter = joint_config_spaces.begin();
        for (; iter < joint_config_spaces.end(); iter++) {
            std::cout << *iter << ", ";
        }
        std::cout << std::endl;

    }

    std::string SystemModel::GetMeshPackageName() const {
        return mesh_package_name_;
    }

    std::string SystemModel::GetMeshPackagePath() const {
        return mesh_package_path_;
    }

    pinocchio::Model& SystemModel::GetPinocchioModel() {
        return model_;
    }
}