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

#include <algorithm>
#include <cstddef>
#include <span>
#include <vector>

namespace motorium::model {

template <typename SCALAR_T>
struct JointStateTpl {
  SCALAR_T q = 0.0;
  SCALAR_T v = 0.0;
  SCALAR_T acceleration = 0.0;
  SCALAR_T effort = 0.0;

  JointStateTpl() = default;
};

template <typename SCALAR_T>
class RobotJointStateTpl : public JointIdMap<JointStateTpl<SCALAR_T>, SCALAR_T> {
  using Base = JointIdMap<JointStateTpl<SCALAR_T>, SCALAR_T>;

 public:
  explicit RobotJointStateTpl(std::size_t joint_count) : Base(joint_count) {}
  explicit RobotJointStateTpl(const RobotDescription& robot_description) : Base(robot_description) {}
};

template <typename SCALAR_T>
struct RootStateTpl {
  VECTOR3_T<SCALAR_T> position_ = VECTOR3_T<SCALAR_T>::Zero();               // In global frame.
  VECTOR3_T<SCALAR_T> linear_velocity_ = VECTOR3_T<SCALAR_T>::Zero();        // in local frame
  QUATERNION_T<SCALAR_T> orientation_ = QUATERNION_T<SCALAR_T>::Identity();  // Root orientation local to world frame
  VECTOR3_T<SCALAR_T> angular_velocity_ = VECTOR3_T<SCALAR_T>::Zero();       // In local frame

  void setConfigurationToZero() {
    position_.setZero();
    orientation_.setIdentity();
    linear_velocity_.setZero();
    angular_velocity_.setZero();
  }
};

template <typename SCALAR_T>
class RobotStateTpl {
 public:
  explicit RobotStateTpl(std::size_t joint_count, std::size_t contact_size = 1)
      : joint_state_map_(joint_count), time_(0.0), contact_flags_(contact_size) {
    setConfigurationToZero();
  }

  explicit RobotStateTpl(const RobotDescription& robot_description, std::size_t contact_size = 1)
      : RobotStateTpl(robot_description.getNumJoints(), contact_size) {}

  // orientation of the root joint wrt world frame, corresponds to the passive
  // rotation from local to world R_l_to_w
  QUATERNION_T<SCALAR_T> getRootRotationLocalToWorldFrame() const { return root_state_.orientation_; }
  VECTOR3_T<SCALAR_T> getRootPositionInWorldFrame() const { return root_state_.position_; }

  VECTOR3_T<SCALAR_T> getRootLinearVelocityInLocalFrame() const { return root_state_.linear_velocity_; }
  VECTOR3_T<SCALAR_T> getRootAngularVelocityInLocalFrame() const { return root_state_.angular_velocity_; }

  // orientation of the root joint wrt world frame, corresponds to the passive
  // rotation from local to world R_l_to_w
  void setRootRotationLocalToWorldFrame(const QUATERNION_T<SCALAR_T>& orientation) { root_state_.orientation_ = orientation; }
  void setRootPositionInWorldFrame(const VECTOR3_T<SCALAR_T>& position) { root_state_.position_ = position; }

  void setRootLinearVelocityInLocalFrame(const VECTOR3_T<SCALAR_T>& linear_velocity) { root_state_.linear_velocity_ = linear_velocity; }
  void setRootAngularVelocityInLocalFrame(const VECTOR3_T<SCALAR_T>& angular_velocity) { root_state_.angular_velocity_ = angular_velocity; }

  void setJointPosition(size_t joint_id, SCALAR_T q) { joint_state_map_.at(joint_id).q = q; }

  SCALAR_T getJointPosition(size_t joint_id) const { return joint_state_map_.at(joint_id).q; }

  void setJointVelocity(size_t joint_id, SCALAR_T v) { joint_state_map_.at(joint_id).v = v; }

  SCALAR_T getJointVelocity(size_t joint_id) const { return joint_state_map_.at(joint_id).v; }

  void setJointAcceleration(size_t joint_id, SCALAR_T acceleration) { joint_state_map_.at(joint_id).acceleration = acceleration; }

  SCALAR_T getJointAcceleration(size_t joint_id) const { return joint_state_map_.at(joint_id).acceleration; }

  void setJointEffort(size_t joint_id, SCALAR_T effort) { joint_state_map_.at(joint_id).effort = effort; }

  SCALAR_T getJointEffort(size_t joint_id) const { return joint_state_map_.at(joint_id).effort; }

  //  Fill a pre-sized vector with joint positions in joint_ids order (real-time safe, no allocation)
  void fillJointPositions(std::span<const joint_index_t> joint_ids, VECTOR_T<SCALAR_T>& out) const {
    joint_state_map_.fillVector(joint_ids, out, [](const JointStateTpl<SCALAR_T>& js) { return js.q; });
  }

  void setJointPositions(std::span<const joint_index_t> joint_ids, const VECTOR_T<SCALAR_T>& positions) {
    joint_state_map_.fromVector(joint_ids, positions, [](JointStateTpl<SCALAR_T>& js, SCALAR_T q) { js.q = q; });
  }

  //  Fill a pre-sized vector with joint velocities in joint_ids order (real-time safe, no allocation)
  void fillJointVelocities(std::span<const joint_index_t> joint_ids, VECTOR_T<SCALAR_T>& out) const {
    joint_state_map_.fillVector(joint_ids, out, [](const JointStateTpl<SCALAR_T>& js) { return js.v; });
  }

  void setJointVelocities(std::span<const joint_index_t> joint_ids, const VECTOR_T<SCALAR_T>& velocities) {
    joint_state_map_.fromVector(joint_ids, velocities, [](JointStateTpl<SCALAR_T>& js, SCALAR_T v) { js.v = v; });
  }

  //  Fill a pre-sized vector with joint accelerations in joint_ids order (real-time safe, no allocation)
  void fillJointAccelerations(std::span<const joint_index_t> joint_ids, VECTOR_T<SCALAR_T>& out) const {
    joint_state_map_.fillVector(joint_ids, out, [](const JointStateTpl<SCALAR_T>& js) { return js.acceleration; });
  }

  void setJointAccelerations(std::span<const joint_index_t> joint_ids, const VECTOR_T<SCALAR_T>& accelerations) {
    joint_state_map_.fromVector(joint_ids, accelerations,
                                [](JointStateTpl<SCALAR_T>& js, SCALAR_T acceleration) { js.acceleration = acceleration; });
  }

  //  Fill a pre-sized vector with joint efforts in joint_ids order (real-time safe, no allocation)
  void fillJointEfforts(std::span<const joint_index_t> joint_ids, VECTOR_T<SCALAR_T>& out) const {
    joint_state_map_.fillVector(joint_ids, out, [](const JointStateTpl<SCALAR_T>& js) { return js.effort; });
  }

  void setJointEfforts(std::span<const joint_index_t> joint_ids, const VECTOR_T<SCALAR_T>& efforts) {
    joint_state_map_.fromVector(joint_ids, efforts, [](JointStateTpl<SCALAR_T>& js, SCALAR_T effort) { js.effort = effort; });
  }

  void setJointState(size_t joint_id, const JointStateTpl<SCALAR_T>& joint_state) { joint_state_map_.at(joint_id) = joint_state; }

  const JointStateTpl<SCALAR_T>& getJointState(size_t joint_id) const { return joint_state_map_.at(joint_id); }

  // Number of joints this state was sized for. The joint accessors above only bounds-check with an
  // MT_DCHECK, so an out-of-range id aborts in debug builds and is undefined behaviour in optimized
  // ones. Callers that cannot guarantee a valid id need this to bounds-check first.
  size_t getNumJoints() const { return joint_state_map_.size(); }

  bool getContactFlag(size_t index) const { return contact_flags_.at(index); }

  void setContactFlag(size_t index, bool contactFlag) { contact_flags_.at(index) = contactFlag; }

  std::vector<bool> getContactFlags() const { return contact_flags_; }

  SCALAR_T getTime() const { return time_; }

  void setTime(SCALAR_T time) { time_ = time; }

  void setConfigurationToZero() {
    root_state_.setConfigurationToZero();

    for (auto& joint : joint_state_map_) {
      joint.q = 0.0;
      joint.v = 0.0;
      joint.acceleration = 0.0;
      joint.effort = 0.0;
    }

    std::fill(contact_flags_.begin(), contact_flags_.end(),
              true);  // Assume robot is in contact
  }

 private:
  RootStateTpl<SCALAR_T> root_state_;
  RobotJointStateTpl<SCALAR_T> joint_state_map_;

  SCALAR_T time_;

  std::vector<bool> contact_flags_;
};

using JointState = JointStateTpl<scalar_t>;
using RootState = RootStateTpl<scalar_t>;
using RobotJointState = RobotJointStateTpl<scalar_t>;
using RobotState = RobotStateTpl<scalar_t>;

}  // namespace motorium::model
