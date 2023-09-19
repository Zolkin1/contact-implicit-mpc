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
    template <typename scalar>
    class SystemModel {
    public:
        SystemModel();

        /**
         * Creates a robot model and sets the joints to the neutral elements.
         * @param model
         */
        SystemModel(const pinocchio::Model& model);

        SystemModel(const pinocchio::Model& model, std::unique_ptr<Controller<scalar>> controller);

        // Get partial derivatives (TBD: which derivatives and how to represent them?)

        // TODO: May want to convert this to a eigen ref argument TBD
        /**
         * Get dynamics - returns the forward dynamics of the robot
         */
        JointVector GetForwardDynamics();

    private:
        /**
         * Calculates the contact forces for the given contact model
         * @return
         */
        pinocchio::container::aligned_vector<pinocchio::ForceTpl<scalar>> GetContactForces();

        pinocchio::Model model_;

        JointVector joint_config_;

        JointVector joint_velocity_;

        std::unique_ptr<Controller<scalar>> controller_;

        // TODO: Probably need to deal with the model data
        //std::unique_ptr<pinocchio::Data> model_data_;
    };
}


#endif //HYBRID_MPC_SYSTEMMODEL_H
