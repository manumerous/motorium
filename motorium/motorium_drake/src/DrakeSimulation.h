/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/fixed_input_port_value.h>

#include <motorium_core/Bounds.h>
#include <motorium_drake/DrakeSimConfig.h>
#include <motorium_model/RobotDescription.h>
#include <motorium_model/RobotJointFeedbackAction.h>
#include <motorium_model/RobotState.h>

namespace motorium::drake_sim {

// The Drake half of DrakeDriver (private to this package). Builds a MultibodyPlant from the model file
// and translates between motorium types, indexed by RobotDescription joint index, and Drake's state and
// actuation vectors. Not thread-safe: DrakeDriver serializes all calls.
//
// The plant uses point contact with the SAP solver. Only the robot's contacts with the ground are
// simulated; collisions between the robot's own bodies are disabled. Joints without an actuator in the
// model get one, with the effort limit of the joint's torque bounds.
class DrakeSimulation {
 public:
  DrakeSimulation(const model::RobotDescription& robot_description, const DrakeSimConfig& config, const std::string& model_path);

  // Sets the floating base pose and velocity (if any) and all joint positions and velocities, and
  // zeroes the applied torques. Simulation time keeps running.
  void setState(const model::RobotState& state);

  // Computes the torque of every actuated joint from `action` at the current state, clamped to the
  // joint's torque bounds (non-finite torques become 0). The torques are applied by the next advance().
  void applyAction(const model::RobotJointFeedbackAction& action);

  // Advances the simulation by one time step. Throws if Drake fails or the state becomes non-finite.
  void advance();

  // Writes the floating base / root body pose and velocity, joint positions, velocities and applied
  // torques (as effort), the simulation time and, if contact bodies are configured, the contact flags.
  void readState(model::RobotState& state) const;

  bool isFloatingBase() const { return is_floating_base_; }
  double rootHeight() const;

 private:
  void buildDiagram(const DrakeSimConfig& config, const std::string& model_path);
  void buildIndexMaps(const DrakeSimConfig& config);

  const model::RobotDescription& robot_description_;
  const double dt_;

  std::unique_ptr<::drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<::drake::systems::Simulator<double>> simulator_;
  const ::drake::multibody::MultibodyPlant<double>* plant_{nullptr};  // owned by diagram_
  ::drake::systems::Context<double>* plant_context_{nullptr};         // owned by simulator_
  ::drake::systems::FixedInputPortValue* actuation_input_{nullptr};
  ::drake::multibody::ModelInstanceIndex model_;
  const ::drake::multibody::RigidBody<double>* root_body_{nullptr};
  bool is_floating_base_{false};

  // Per RobotDescription joint: index of its position and velocity in Drake's state x = [q; v].
  std::vector<int> joint_q_index_;
  std::vector<int> joint_v_index_;

  // Per actuator, in the order of the actuation input port: the actuated joint and its torque bounds.
  std::vector<joint_index_t> actuated_joints_;
  std::vector<Bounds> torque_bounds_;
  Eigen::VectorXd actuation_;  // currently applied torques

  std::vector<::drake::multibody::BodyIndex> contact_bodies_;

  // Current simulation time is step_index_ * dt_. See advance() for why time is not accumulated.
  int64_t step_index_{0};
};

}  // namespace motorium::drake_sim
