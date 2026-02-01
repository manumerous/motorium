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
#include <motorium_core/Types.h>
#include <motorium_model/JointIDMap.h>
#include <motorium_model/RobotDescription.h>

namespace motorium::model {

struct JointState {
  scalar_t q = 0.0;
  scalar_t v = 0.0;
  scalar_t effort = 0.0;

  JointState() = default;
};

class RobotJointState : public JointIdMap<JointState> {
 public:
  explicit RobotJointState(const RobotDescription& robot_description) : JointIdMap(robot_description) {}
};

struct RootState {
  vector3_t position_ = vector3_t::Zero();               // In global frame.
  vector3_t linear_velocity_ = vector3_t::Zero();        // in local frame
  quaternion_t orientation_ = quaternion_t::Identity();  // Root orientation local to world frame
  vector3_t angular_velocity_ = vector3_t::Zero();       // In local frame

  void setConfigurationToZero() {
    position_.setZero();
    orientation_.setIdentity();
    linear_velocity_.setZero();
    angular_velocity_.setZero();
  }
};
class RobotState {
 public:
  RobotState(const RobotDescription& robot_description, size_t contactSize = 1);

  // orientation of the root joint wrt world frame, corresponds to the passive
  // rotation from local to world R_l_to_w
  quaternion_t getRootRotationLocalToWorldFrame() const { return root_state_.orientation_; }
  vector3_t getRootPositionInWorldFrame() const { return root_state_.position_; }

  vector3_t getRootLinearVelocityInLocalFrame() const { return root_state_.linear_velocity_; }
  vector3_t getRootAngularVelocityInLocalFrame() const { return root_state_.angular_velocity_; }

  // orientation of the root joint wrt world frame, corresponds to the passive
  // rotation from local to world R_l_to_w
  void setRootRotationLocalToWorldFrame(const quaternion_t& orientation) { root_state_.orientation_ = orientation; }
  void setRootPositionInWorldFrame(const vector3_t& position) { root_state_.position_ = position; }

  void setRootLinearVelocityInLocalFrame(const vector3_t& linear_velocity) { root_state_.linear_velocity_ = linear_velocity; }
  void setRootAngularVelocityInLocalFrame(const vector3_t& angular_velocity) { root_state_.angular_velocity_ = angular_velocity; }

  void setJointPosition(size_t joint_id, scalar_t joint_position) { joint_state_map_.at(joint_id).q = joint_position; }

  scalar_t getJointPosition(size_t joint_id) const { return joint_state_map_.at(joint_id).q; }

  void setJointVelocity(size_t joint_id, scalar_t jointVelocity) { joint_state_map_.at(joint_id).v = jointVelocity; }

  scalar_t getJointVelocity(size_t joint_id) const { return joint_state_map_.at(joint_id).v; }

  void setJointEffort(size_t joint_id, scalar_t jointEffort) { joint_state_map_.at(joint_id).effort = jointEffort; }

  scalar_t getJointEffort(size_t joint_id) const { return joint_state_map_.at(joint_id).effort; }

  //  Get a vector_t of joint positions given a vector of joint IDs

  vector_t getJointPositions(std::vector<joint_index_t> joint_ids) const {
    return joint_state_map_.toVector(joint_ids, [](const JointState& js) { return js.q; });
  }

  //  Get a vector_t of joint velocities given a vector of joint IDs
  vector_t getJointVelocities(std::vector<joint_index_t> joint_ids) const {
    return joint_state_map_.toVector(joint_ids, [](const JointState& js) { return js.v; });
  }

  void setJointState(size_t joint_id, const JointState& joint_state) { joint_state_map_.at(joint_id) = joint_state; }

  const JointState& getJointState(size_t joint_id) const { return joint_state_map_.at(joint_id); }

  bool getContactFlag(size_t index) const { return contact_flags_.at(index); }

  void setContactFlag(size_t index, bool contactFlag) { contact_flags_.at(index) = contactFlag; }

  std::vector<bool> getContactFlags() const { return contact_flags_; }

  scalar_t getTime() const { return time_; }

  void setTime(scalar_t time) { time_ = time; }

  void setConfigurationToZero();

 private:
  RootState root_state_;
  RobotJointState joint_state_map_;

  scalar_t time_;

  std::vector<bool> contact_flags_;
};

}  // namespace motorium::model
