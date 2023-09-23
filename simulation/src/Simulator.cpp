//
// Created by zach on 9/21/23.
// Copyright (c) 2023 Zachary Olkin. All rights reserved.
//

#include <cmath>

#include "Simulator.h"
#include <drake/multibody/parsing/parser.h>
#include <drake/visualization/visualization_config_functions.h>

namespace cimpc {
    namespace simulator {
        Simulator::Simulator(const SystemModel& model) :
                meshcat_(new drake::geometry::Meshcat),
                meshcat_viz_(meshcat_) {
            drake::systems::DiagramBuilder<double> builder;

            std::tie(plant_, scene_graph_) = drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0);

            drake::multibody::Parser parser(plant_, scene_graph_);

            if (model.GetMeshPackageName() != "") {
                drake::multibody::PackageMap& package_map = parser.package_map();
                package_map.Add(model.GetMeshPackageName(),  model.GetMeshPackagePath());
            }

            parser.AddModelFromFile(model.GetURDFFilePath());

            // For now, finalize here
            plant_->Finalize();

            std::vector<std::string> position_names = plant_->GetStateNames();
            auto iter = position_names.begin();
            for (; iter < position_names.end(); iter++) {
                std::cout << *iter << ", ";
            }

            std::cout << std::endl;

            drake::visualization::AddDefaultVisualization(&builder, meshcat_);

            diagram_ = builder.Build();

            std::unique_ptr<drake::systems::Context<double>> diagram_context = diagram_->CreateDefaultContext();
            diagram_->SetDefaultContext(diagram_context.get());

            drake::systems::Context<double>& plant_context = diagram_->GetMutableSubsystemContext(*plant_,
                                                                                                  diagram_context.get());

            simulator_ = std::make_unique<drake::systems::Simulator<double>>(*diagram_, std::move(diagram_context));

            InitializeSimulator();
        }

        void Simulator::InitializeSimulator() {
            simulator_->set_publish_every_time_step(false);
            simulator_->set_target_realtime_rate(1.0);
            simulator_->Initialize();
        }


        /*
         * Drake position is:
         * [qw, qx, qy, qz, x, y, z, q1, ... ,qN]
         * Pinocchio configuration is:
         * [x, y, z, q*, q*, q*, q*, q1, ..., qN] (I don't know the order of the quaternion)
         * TODO: determine the starting angles in drake relative to pinocchio
         */
        void Simulator::SetState(JointVector pose, double time) {
            // Will need to convert to the proper drake vector

            // JointVector is just a vector of configurations that corresponds to the configurations used in pinocchio
            // need to check the configuration representation in drake then convert between the two.

            // Try to convert poses via acos(q_i) for i > 6

            JointVector q = Eigen::VectorXd::Zero(plant_->num_positions());
            // TODO: Make this stuff not hard coded
            int j = 3;
            for (int i = 4; i < pose.size(); i+=2) {
                q[j] = acos(pose[i]);
                std::cout << q[j] << ", ";
                j++;
            }

            // Assign the floating base values as they are in pinocchio
            j = 4;
            for (int i = 0; i < 3; i++) {
                q[j] = pose[i];
                j++;
            }


            std::unique_ptr<drake::systems::Context<double>> diagram_context = diagram_->CreateDefaultContext();

            // Set the state by creating a new context
            drake::systems::Context<double>& plant_context = diagram_->GetMutableSubsystemContext(*plant_,
                                                                                                  diagram_context.get());
            plant_->SetPositions(&plant_context, q);
            // TODO: Update the time for the meshcat animation
            diagram_context->SetTime(time);


            simulator_->reset_context(std::move(diagram_context));

            // Then re-initialize the simulator (I believe this will then update the visualizer)
            simulator_->Initialize();

            std::cout << "Simulator re-initialized" << std::endl;
        }

        int Simulator::GetNumStates() const {
            return simulator_->get_context().num_total_states();
        }

        int Simulator::GetNumContinuousStates() const {
            return simulator_->get_context().num_continuous_states();
        }

        void Simulator::StartRecording(double frame_rate) {
            drake::geometry::MeshcatAnimation* animation = meshcat_viz_.StartRecording();
            // Set the frame rate
            animation->frame(frame_rate);
        }

        void Simulator::StopRecording() {
            meshcat_viz_.StopRecording();
            meshcat_viz_.PublishRecording();
        }

    }
}