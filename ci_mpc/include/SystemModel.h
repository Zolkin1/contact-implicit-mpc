//
// Created by zach on 9/18/23.
// Copyright (c) 2023 Zachary Olkin. All rights reserved.
//

#ifndef HYBRID_MPC_SYSTEMMODEL_H
#define HYBRID_MPC_SYSTEMMODEL_H

#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/multibody/data.hpp"

#include "Controller.h"

// For now I am assuming that a double will work
// if I need a templated type, i.e. for autodiff, then I will add that in later


namespace cimpc {

    // TODO: Can I do this with fixed size vectors to avoid the memory allocation?
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> JointVector;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> InputVector;

    /**
     * SystemModel
     * @tparam scalar
     *
     * Holds all the information related to the robot model: pinocchio model, current robot state, etc...
     */
    class SystemModel {
    public:
        SystemModel();

        //explicit SystemModel(const pinocchio::Model& model);

        explicit SystemModel(const pinocchio::Model& model, std::unique_ptr<Controller> controller = nullptr);

        explicit SystemModel(const std::string& file_path, const bool& is_floating_base, std::unique_ptr<Controller> controller = nullptr);

        // Get partial derivatives (TBD: which derivatives and how to represent them?)

        // TODO: May want to convert this to a eigen ref argument TBD
        /**
         * Get dynamics - returns the forward dynamics of the robot
         */
        JointVector GetForwardDynamics();

        JointVector GetPose();

        JointVector GetJointVelocity();

        void SetPose(const JointVector& joint_config);

        void SetJointVelocity(const JointVector& joint_velocity);

        void SetMeshPackageInformation(std::string mesh_package_name, std::string mesh_package_path);

        void PrintState() const;

        std::string GetURDFFilePath() const;

        /**
         * Gets the configuration dimension used by pinocchio.
         * Note that continuous joints (i.e. revolute joints with no limits) have configuration dimension 2.
         * This is because pinocchio store the configuration as (cos(theta), sin(theta)) and still uses velocity of
         * theta_dot (SO(2) for the configuration)
         * @return number of configuration dimensions
         */
        int GetConfigurationDimension() const;

        int GetVelocityDimension() const;

        void PrintSystemInfo() const;

        std::string GetMeshPackageName() const;

        std::string GetMeshPackagePath() const;

        // TODO: Do i need to make this return a const model
        pinocchio::Model& GetPinocchioModel();

    private:
        /**
         * Calculates the contact forces for the given contact model
         * @return
         */
        pinocchio::container::aligned_vector<pinocchio::ForceTpl<double>> GetContactForces();

        pinocchio::Model model_;

        JointVector joint_config_;

        JointVector joint_velocity_;

        std::unique_ptr<Controller> controller_;

        std::string urdf_file_path_;

        std::string mesh_package_name_;

        std::string mesh_package_path_;

        bool is_floating_base_;

        // TODO: Need to think about how I want to generate this data
        // depending on how the data is generated I can make it very efficient by making sure that the proper data
        // is generated in the correct order. If this does not happen I can easily be re-calculating things
        // Maybe an internal state to keep track of such things?
        std::unique_ptr<pinocchio::Data> model_data_;
    };
}


#endif //HYBRID_MPC_SYSTEMMODEL_H
