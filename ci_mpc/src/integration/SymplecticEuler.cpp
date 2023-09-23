//
// Created by zach on 9/18/23.
// Copyright (c) 2023 Zachary Olkin. All rights reserved.
//

#include "SymplecticEuler.h"
#include "pinocchio/multibody/liegroup/liegroup-base.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

namespace cimpc {
    namespace integration {
        SymplecticEuler::SymplecticEuler(double dt) : Integrator(dt) {

        }

        JointVector SymplecticEuler::Advance(SystemModel& model, double time) {
            JointVector q = model.GetPose();
            JointVector dq = model.GetJointVelocity();

            // Integrate
            // dq_kp1 = dq_k + dt*ddq
            // q_kp1 = q_k + dt*dq_kp1

            int N = std::floor(time / this->dt_);
            for (int i = 0; i < N; i++) {

                // Get the forward dynamics
                JointVector ddq = model.GetForwardDynamics();

                dq += this->dt_*ddq;

                // pinocchio::integrate is needed to transform the tangent vector into the configuration space
                // this happens because the configuration space can be a member of a lie group. The integration
                // is just performing the exponential map (I believe) which is effectively doing q = q + 1*v where
                // v is mapped into the lie group from the tangent space. This aligns with how jiminey works.
                JointVector q_out = Eigen::VectorXd::Zero(q.size());
                pinocchio::integrate(model.GetPinocchioModel(), q, this->dt_*dq, q_out);
                q = q_out;
            }

            model.SetPose(q);
            model.SetJointVelocity(dq);

            return q;
        }

    }
}