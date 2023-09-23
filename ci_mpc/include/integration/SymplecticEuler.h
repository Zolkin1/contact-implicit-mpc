//
// Created by zach on 9/18/23.
// Copyright (c) 2023 Zachary Olkin. All rights reserved.
//

#ifndef HYBRID_MPC_SYMPLECTICEULER_H
#define HYBRID_MPC_SYMPLECTICEULER_H

// TODO: Change this include file
#include "pinocchio/parsers/sample-models.hpp"

#include "Integrator.h"

namespace cimpc {
    namespace integration {
        class SymplecticEuler final : public Integrator {
        public:
            SymplecticEuler(double dt);

            JointVector Advance(SystemModel& model, double time);

        private:
        };
    } // integration
} // cimpc


#endif //HYBRID_MPC_SYMPLECTICEULER_H
