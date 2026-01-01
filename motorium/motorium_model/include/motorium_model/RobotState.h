#pragma once
#include <motorium_core/Types.h>
#include <motorium_model/JointIDMap.h>
#include <motorium_model/RobotDescription.h>

namespace motorium::model {

struct JointState {
  scalar_t position = 0.0;
  scalar_t velocity = 0.0;
  scalar_t measuredEffort = 0.0;

  JointState() = default;
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
/**
 * Construct a RobotState for the given robot description and number of contact points.
 * @param robot_description Description of the robot used to size and initialize internal state.
 * @param contactSize Number of contact flags to allocate.
 */
/**
 * Get the root orientation that rotates vectors from the local (root) frame to the world frame.
 * @returns The root orientation as a quaternion representing local-to-world rotation.
 */
/**
 * Get the root position expressed in the world frame.
 * @returns The root position vector in world coordinates.
 */
/**
 * Get the root linear velocity expressed in the local (root) frame.
 * @returns The root linear velocity vector in the local frame.
 */
/**
 * Get the root angular velocity expressed in the local (root) frame.
 * @returns The root angular velocity vector in the local frame.
 */
/**
 * Set the root orientation that rotates vectors from the local (root) frame to the world frame.
 * @param orientation Quaternion representing the local-to-world rotation to store as the root orientation.
 */
/**
 * Set the root position expressed in the world frame.
 * @param position Root position vector in world coordinates to store.
 */
/**
 * Set the root linear velocity expressed in the local (root) frame.
 * @param linear_velocity Linear velocity vector in the local frame to store.
 */
/**
 * Set the root angular velocity expressed in the local (root) frame.
 * @param angular_velocity Angular velocity vector in the local frame to store.
 */
/**
 * Set the position of a single joint.
 * @param joint_id Index of the joint to update.
 * @param joint_position Position value to assign to the joint.
 */
/**
 * Get the position of a single joint.
 * @param joint_id Index of the joint to query.
 * @returns The joint position value.
 */
/**
 * Set the velocity of a single joint.
 * @param joint_id Index of the joint to update.
 * @param jointVelocity Velocity value to assign to the joint.
 */
/**
 * Get the velocity of a single joint.
 * @param joint_id Index of the joint to query.
 * @returns The joint velocity value.
 */
/**
 * Get a vector of joint positions for the specified joint IDs.
 * @param joint_ids Vector of joint indices to collect positions for.
 * @returns A vector_t containing the positions of the requested joints in the same order as joint_ids.
 */
/**
 * Get a vector of joint velocities for the specified joint IDs.
 * @param joint_ids Vector of joint indices to collect velocities for.
 * @returns A vector_t containing the velocities of the requested joints in the same order as joint_ids.
 */
/**
 * Replace the stored JointState for a specific joint.
 * @param joint_id Index of the joint to update.
 * @param joint_state JointState to assign to the joint.
 */
/**
 * Get the JointState for a specific joint.
 * @param joint_id Index of the joint to retrieve.
 * @returns The JointState stored for the specified joint.
 */
/**
 * Get the contact flag at the given index.
 * @param index Index of the contact flag to query.
 * @returns `true` if contact is active at the index, `false` otherwise.
 */
/**
 * Set the contact flag at the given index.
 * @param index Index of the contact flag to set.
 * @param contactFlag Boolean value to assign to the contact flag.
 */
/**
 * Get all contact flags.
 * @returns A copy of the vector of contact flags.
 */
/**
 * Get the stored time stamp.
 * @returns The current time value stored in the state.
 */
/**
 * Set the stored time stamp.
 * @param time Time value to store in the state.
 */
/**
 * Reset the root pose, root velocities, all joint states, contact flags, and time to their zero/default values.
 */
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

  void setJointPosition(size_t joint_id, scalar_t joint_position) { joint_state_map_.at(joint_id).position = joint_position; }

  scalar_t getJointPosition(size_t joint_id) const { return joint_state_map_.at(joint_id).position; }

  void setJointVelocity(size_t joint_id, scalar_t jointVelocity) { joint_state_map_.at(joint_id).velocity = jointVelocity; }

  scalar_t getJointVelocity(size_t joint_id) const { return joint_state_map_.at(joint_id).velocity; }

  //  Get a vector_t of joint positions given a vector of joint IDs

  vector_t getJointPositions(std::vector<joint_index_t> joint_ids) const {
    return joint_state_map_.toVector(joint_ids, [](const JointState& js) { return js.position; });
  }

  //  Get a vector_t of joint velocities given a vector of joint IDs
  vector_t getJointVelocities(std::vector<joint_index_t> joint_ids) const {
    return joint_state_map_.toVector(joint_ids, [](const JointState& js) { return js.velocity; });
  }

  void setJointState(size_t joint_id, const JointState& joint_state) { joint_state_map_.at(joint_id) = joint_state; }

  JointState getJointState(size_t joint_id) const { return joint_state_map_.at(joint_id); }

  bool getContactFlag(size_t index) const { return contact_flags_.at(index); }

  void setContactFlag(size_t index, bool contactFlag) { contact_flags_.at(index) = contactFlag; }

  std::vector<bool> getContactFlags() const { return contact_flags_; }

  scalar_t getTime() const { return time_; }

  void setTime(scalar_t time) { time_ = time; }

  void setConfigurationToZero();

 private:
  RootState root_state_;
  JointIdMap<JointState> joint_state_map_;

  scalar_t time_;

  std::vector<bool> contact_flags_;
};

}  // namespace motorium::model