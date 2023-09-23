//
// Created by zach on 9/21/23.
// Copyright (c) 2023 Zachary Olkin. All rights reserved.
//

#ifndef HYBRID_MPC_SIMULATOR_H
#define HYBRID_MPC_SIMULATOR_H

#include <drake/systems/analysis/simulator.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/multibody/plant/multibody_plant.h>

#include <SystemModel.h>

// TODO: Need to make sure that this is designed so that:
// - All the heavy overhead can be done before the real time part (should just be recording during real time stuff)
// - This class can be not used all all for hardware deployment

namespace cimpc {
    namespace simulator {
        /**
         * Handle all the interface with meshcat and drake for internal simulation purposes
         */
        class Simulator {
        public:
            Simulator(const SystemModel& model);

            void InitializeSimulator();

            /**
             * Sets the state of the simulated robot. Also updates the visualization
             */
            void SetState(JointVector pose, double time = 0);

            int GetNumStates() const;

            int GetNumContinuousStates() const;

            /**
             * Starts a meshcat visualization recording
             */
            void StartRecording(double frame_rate);

            /**
             * Stops the recording and sends it to meshcat
             */
            void StopRecording();


        protected:
        private:
            std::unique_ptr<drake::systems::Diagram<double>> diagram_;

            drake::multibody::MultibodyPlant<double>* plant_;
            drake::geometry::SceneGraph<double>* scene_graph_;

            std::shared_ptr<drake::geometry::Meshcat> meshcat_;
            drake::geometry::MeshcatVisualizer<double> meshcat_viz_;

            std::unique_ptr<drake::systems::Simulator<double>> simulator_;

            //std::unique_ptr<drake::systems::Context<double>> diagram_context_;

        };
    }
}


#endif //HYBRID_MPC_SIMULATOR_H
