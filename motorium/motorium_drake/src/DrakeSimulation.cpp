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

#include "DrakeSimulation.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

#include <drake/geometry/collision_filter_declaration.h>
#include <drake/geometry/scene_graph.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/math/spatial_algebra.h>
#include <drake/multibody/plant/contact_results.h>
#include <drake/systems/framework/diagram_builder.h>

#include <motorium_core/Check.h>

#include "DrakeModelLoading.h"

namespace motorium::drake_sim {

namespace dmb = ::drake::multibody;
namespace dsys = ::drake::systems;

DrakeSimulation::DrakeSimulation(const model::RobotDescription& robot_description,
                                 const DrakeSimConfig& config,
                                 const std::string& model_path)
    : robot_description_(robot_description), dt_(config.dt) {
  buildDiagram(config, model_path);
  buildIndexMaps(config);
}

void DrakeSimulation::buildDiagram(const DrakeSimConfig& config, const std::string& model_path) {
  dsys::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = dmb::AddMultibodyPlantSceneGraph(&builder, config.dt);
  plant.set_contact_model(dmb::ContactModel::kPointContactOnly);
  plant.set_discrete_contact_approximation(dmb::DiscreteContactApproximation::kSap);

  model_ = internal::addRobotModel(plant, config.packagePaths, model_path);

  for (const std::string& joint_name : robot_description_.getJointNames()) {
    MT_CHECK(plant.HasJointNamed(joint_name, model_))
        << "[DrakeDriver] Joint '" << joint_name << "' of the RobotDescription is not in the Drake model " << model_path;
    dmb::Joint<double>& joint = plant.GetMutableJointByName(joint_name, model_);
    MT_CHECK(joint.num_velocities() == 1) << "[DrakeDriver] Joint '" << joint_name << "' must be revolute or prismatic, it is a "
                                          << joint.type_name() << " joint";
    joint.set_default_damping_vector(Eigen::VectorXd::Constant(1, config.defaultJointDamping));

    // URDFs without <transmission> tags have no actuators.
    if (!plant.HasJointActuatorNamed(joint_name, model_)) {
      const Bounds& torque_bounds = robot_description_.getJointDescription(joint_name).torque_bounds;
      plant.AddJointActuator(joint_name, joint, std::max(std::abs(torque_bounds.min), std::abs(torque_bounds.max)));
    }
  }

  internal::addGround(plant, config.ground, /*with_collision=*/true);
  scene_graph.collision_filter_manager().Apply(::drake::geometry::CollisionFilterDeclaration().ExcludeWithin(
      plant.CollectRegisteredGeometries(internal::collectModelBodies(plant, model_))));

  plant.Finalize();
  root_body_ = &internal::findRootBody(plant, model_, is_floating_base_);

  plant_ = &plant;
  diagram_ = builder.Build();
  simulator_ = std::make_unique<dsys::Simulator<double>>(*diagram_);
  plant_context_ = &plant.GetMyMutableContextFromRoot(&simulator_->get_mutable_context());

  actuation_ = Eigen::VectorXd::Zero(plant.num_actuated_dofs(model_));
  actuation_input_ = &plant.get_actuation_input_port(model_).FixValue(plant_context_, actuation_);

  if (config.verbose) {
    std::cerr << "[DrakeDriver] Loaded " << model_path << ": " << plant.num_positions() << " positions, " << plant.num_velocities()
              << " velocities, " << actuation_.size() << " actuators, root body '" << root_body_->name() << "'"
              << (is_floating_base_ ? " (floating)" : " (fixed)") << std::endl;
  }
}

void DrakeSimulation::buildIndexMaps(const DrakeSimConfig& config) {
  const int num_positions = plant_->num_positions();
  for (size_t i = 0; i < robot_description_.getNumJoints(); ++i) {
    const dmb::Joint<double>& joint = plant_->GetJointByName(robot_description_.getJointName(i), model_);
    joint_q_index_.push_back(joint.position_start());
    joint_v_index_.push_back(num_positions + joint.velocity_start());
  }

  for (const dmb::JointActuatorIndex& actuator_index : plant_->GetJointActuatorIndices(model_)) {
    const dmb::JointActuator<double>& actuator = plant_->get_joint_actuator(actuator_index);
    const std::string& joint_name = actuator.joint().name();
    MT_CHECK(robot_description_.containsJoint(joint_name))
        << "[DrakeDriver] Actuator '" << actuator.name() << "' drives joint '" << joint_name << "', which is not in the RobotDescription";
    actuated_joints_.push_back(robot_description_.getJointIndex(joint_name));
    torque_bounds_.push_back(robot_description_.getJointDescription(joint_name).torque_bounds);
  }

  for (const std::string& body_name : config.contactBodyNames) {
    MT_CHECK(plant_->HasBodyNamed(body_name, model_)) << "[DrakeDriver] Contact body '" << body_name << "' is not in the model";
    contact_bodies_.push_back(plant_->GetBodyByName(body_name, model_).index());
  }
}

void DrakeSimulation::setState(const model::RobotState& state) {
  if (is_floating_base_) {
    const quaternion_t orientation = state.getRootRotationLocalToWorldFrame().normalized();
    plant_->SetFreeBodyPose(plant_context_, *root_body_,
                            ::drake::math::RigidTransformd(::drake::math::RotationMatrixd(orientation.toRotationMatrix()),
                                                           state.getRootPositionInWorldFrame()));
    // RobotState root velocities are in the local frame, Drake's spatial velocity in the world frame.
    plant_->SetFreeBodySpatialVelocity(plant_context_, *root_body_,
                                       dmb::SpatialVelocity<double>(orientation * state.getRootAngularVelocityInLocalFrame(),
                                                                    orientation * state.getRootLinearVelocityInLocalFrame()));
  }

  Eigen::VectorXd x = plant_->GetPositionsAndVelocities(*plant_context_);
  for (size_t i = 0; i < joint_q_index_.size(); ++i) {
    x(joint_q_index_[i]) = state.getJointPosition(i);
    x(joint_v_index_[i]) = state.getJointVelocity(i);
  }
  plant_->SetPositionsAndVelocities(plant_context_, x);

  actuation_.setZero();
  actuation_input_->GetMutableVectorData<double>()->SetFromVector(actuation_);

  simulator_->Initialize();
  step_index_ = std::llround(simulator_->get_context().get_time() / dt_);
}

void DrakeSimulation::applyAction(const model::RobotJointFeedbackAction& action) {
  const auto x = plant_->GetPositionsAndVelocities(*plant_context_);
  for (size_t k = 0; k < actuated_joints_.size(); ++k) {
    const joint_index_t joint = actuated_joints_[k];
    const double torque = action.at(joint).getTotalFeedbackTorque(x(joint_q_index_[joint]), x(joint_v_index_[joint]));
    // Drake does not limit the actuation input to the actuator's effort limit, so clamp here.
    actuation_(static_cast<Eigen::Index>(k)) = std::isfinite(torque) ? torque_bounds_[k].clamp(torque) : 0.0;
  }
  actuation_input_->GetMutableVectorData<double>()->SetFromVector(actuation_);
}

void DrakeSimulation::advance() {
  // Advance to (n + 1) * dt, computed by multiplication. Accumulating t += dt drifts past Drake's update
  // times within ~10 steps; one AdvanceTo() then performs two updates and the state runs a step ahead.
  simulator_->AdvanceTo(static_cast<double>(step_index_ + 1) * dt_);
  ++step_index_;

  if (!plant_->GetPositionsAndVelocities(*plant_context_).allFinite()) {
    throw std::runtime_error("Drake simulation state became non-finite");
  }
}

void DrakeSimulation::readState(model::RobotState& state) const {
  const ::drake::math::RigidTransformd& X_WB = plant_->EvalBodyPoseInWorld(*plant_context_, *root_body_);
  const quaternion_t orientation(X_WB.rotation().matrix());
  state.setRootPositionInWorldFrame(X_WB.translation());
  state.setRootRotationLocalToWorldFrame(orientation);

  const dmb::SpatialVelocity<double>& V_WB = plant_->EvalBodySpatialVelocityInWorld(*plant_context_, *root_body_);
  state.setRootLinearVelocityInLocalFrame(orientation.conjugate() * V_WB.translational());
  state.setRootAngularVelocityInLocalFrame(orientation.conjugate() * V_WB.rotational());

  const auto x = plant_->GetPositionsAndVelocities(*plant_context_);
  for (size_t i = 0; i < joint_q_index_.size(); ++i) {
    state.setJointPosition(i, x(joint_q_index_[i]));
    state.setJointVelocity(i, x(joint_v_index_[i]));
  }
  for (size_t k = 0; k < actuated_joints_.size(); ++k) {
    state.setJointEffort(actuated_joints_[k], actuation_(static_cast<Eigen::Index>(k)));
  }
  state.setTime(plant_context_->get_time());

  if (contact_bodies_.empty()) return;
  for (size_t c = 0; c < contact_bodies_.size(); ++c) {
    state.setContactFlag(c, false);
  }
  const auto& contacts = plant_->get_contact_results_output_port().Eval<dmb::ContactResults<double>>(*plant_context_);
  for (int i = 0; i < contacts.num_point_pair_contacts(); ++i) {
    const auto& contact = contacts.point_pair_contact_info(i);
    for (size_t c = 0; c < contact_bodies_.size(); ++c) {
      if (contact.bodyA_index() == contact_bodies_[c] || contact.bodyB_index() == contact_bodies_[c]) {
        state.setContactFlag(c, true);
      }
    }
  }
}

double DrakeSimulation::rootHeight() const {
  return plant_->EvalBodyPoseInWorld(*plant_context_, *root_body_).translation().z();
}

}  // namespace motorium::drake_sim
