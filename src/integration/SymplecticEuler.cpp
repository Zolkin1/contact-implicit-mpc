//
// Created by zach on 9/18/23.
// Copyright (c) 2023 Zachary Olkin. All rights reserved.
//

#include "SymplecticEuler.h"

namespace cimpc {

    template <typename scalar>
    SymplecticEuler<scalar>::SymplecticEuler() : Integrator<scalar>() {

    }

    template <typename scalar>
    scalar SymplecticEuler<scalar>::AdvanceTo(SystemModel<scalar> model, double time) {
        // Get the forward dynamics

        PINOCCHIO_ALIGNED_STD_VECTOR(pinocchio::Force) external_force;

        JointVector ddq = model.GetForwardDynamics(external_force);

        // Get the derivative for the velocity


        // On loop perform the desired Euler step
    }
}