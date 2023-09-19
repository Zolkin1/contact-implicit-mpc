//
// Created by zach on 9/18/23.
// Copyright (c) 2023 Zachary Olkin. All rights reserved.
//

#ifndef HYBRID_MPC_INTEGRATOR_H
#define HYBRID_MPC_INTEGRATOR_H

#include "SystemModel.h"
#include "pinocchio/container/aligned-vector.hpp"

// TODO: template this?
// Might be able to hardcode the derivative through the integration and then call the code-gen'd derivative of the dynamics
// Need to be able to integrate vector quantities
// Build foundation on Eigen

namespace cimpc {

    // I want this type to be the state vector
    template <typename T>
    class Integrator {
    public:
        // Integrator needs to take in a system model object
        Integrator();

        // TODO: Do i need to return derivatives here?
        /**
         * Integrate the system forward to the desired time
         * @param time - time to integrate forward
         * @return final state of the system
         */
        virtual T AdvanceTo(SystemModel<T> model, double time) = 0;

    private:
        // Initial condition

    };

}   // namespace


#endif //HYBRID_MPC_INTEGRATOR_H
