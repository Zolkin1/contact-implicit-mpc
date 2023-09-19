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
    template <typename scalar>
    class SymplecticEuler final : public Integrator<scalar> {
    public:
        SymplecticEuler();

        scalar AdvanceTo(SystemModel<scalar> model, double time);
    private:
    };
}


#endif //HYBRID_MPC_SYMPLECTICEULER_H
