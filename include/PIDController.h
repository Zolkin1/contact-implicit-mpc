//
// Created by zach on 9/19/23.
// Copyright (c) 2023 Zachary Olkin. All rights reserved.
//

#ifndef HYBRID_MPC_PIDCONTROLLER_H
#define HYBRID_MPC_PIDCONTROLLER_H

#include "Controller.h"

namespace cimpc {
    template <typename scalar>
    class PIDController : public Controller<scalar> {
    public:
        PIDController();

        InputVector GetControl(const JointVector& joint_config, const JointVector& joint_velocity);

    private:

    };
}


#endif //HYBRID_MPC_PIDCONTROLLER_H
