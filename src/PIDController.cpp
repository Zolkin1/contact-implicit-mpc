//
// Created by zach on 9/19/23.
// Copyright (c) 2023 Zachary Olkin. All rights reserved.
//

#include <iostream>

#include "PIDController.h"

namespace cimpc {
    template <typename scalar>
    PIDController<scalar>::PIDController() {
        std::cout << "in the PID controller constructor" << std::endl;
    }

    // TODO: Make this a proper implementation
    template <typename scalar>
    InputVector PIDController<scalar>::GetControl(const JointVector& joint_config,
                                                  const JointVector& joint_velocity) {
        InputVector output = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(joint_config.rows());

        return output;
    }
}