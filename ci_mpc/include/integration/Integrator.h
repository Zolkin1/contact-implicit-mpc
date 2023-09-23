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
    namespace integration {

        // I want this type to be the state vector
        class Integrator {
        public:
            // Integrator needs to take in a system model object
            Integrator(double dt);

            // TODO: Do i need to return derivatives here?
            /**
             * Integrate the system forward to the desired time.
             * The model is updated to the final state before the return.
             * @param time - time to integrate forward
             * @return final joint configuration of the system
             */
            virtual JointVector Advance(SystemModel& model, double time) = 0;

        protected:
            double dt_;

        private:
        };
    } // integration
}   // cimpc


#endif //HYBRID_MPC_INTEGRATOR_H
