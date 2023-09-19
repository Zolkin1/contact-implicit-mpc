//
// Created by zach on 9/19/23.
// Copyright (c) 2023 Zachary Olkin. All rights reserved.
//

#ifndef HYBRID_MPC_CONTROLLER_H
#define HYBRID_MPC_CONTROLLER_H

#include <Eigen/Core>

namespace cimpc {
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> JointVector;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> InputVector;

    /**
     * Abstract controller class
     */
    template <typename scalar>
    class Controller {
    public:
        Controller();

        virtual InputVector GetControl(const JointVector& joint_config, const JointVector& joint_velocity) = 0;
    private:

    };
}


#endif //HYBRID_MPC_CONTROLLER_H
