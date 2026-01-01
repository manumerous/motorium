#pragma once

#include <motorium_core/Bounds.h>
#include <motorium_core/Types.h>

#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"

/**
 * Describes a single joint including its name and allowable position, velocity, and torque bounds.
 */

/**
 * Validate that the joint description is well-formed (e.g., name and bounds are consistent).
 */

/**
 * Write a human-readable representation of `joint` to `os`.
 * @param os Output stream to write to.
 * @param joint JointDescription to format.
 * @returns Reference to the provided output stream.
 */

/**
 * Construct a RobotDescription from a URDF file path.
 * @param urdfPath Filesystem path to a URDF describing the robot.
 */

/**
 * Construct a RobotDescription from an explicit list of joint descriptions.
 * @param jointDescriptions Vector of joint descriptions defining the robot's joints and ordering.
 */

/**
 * Return the indices corresponding to `joint_names` in the same order.
 * @param joint_names Ordered span of joint names to look up.
 * @returns Vector of joint indices matching the requested names.
 */

/**
 * Return the set of joint names in index order.
 * @returns Const reference to the vector of joint names.
 */

/**
 * Check whether a joint with the given name exists in this description.
 * @param jointName Name of the joint to check.
 * @returns `true` if the joint exists, `false` otherwise.
 */

/**
 * Return the JointDescription for the joint with the given name.
 * @param joint_name Name of the joint to retrieve.
 * @returns Const reference to the JointDescription for `joint_name`.
 */

/**
 * Return the JointDescription for the joint at the given index.
 * @param joint_index Index of the joint to retrieve.
 * @returns Const reference to the JointDescription at `joint_index`.
 */

/**
 * Return the number of joints described by this RobotDescription.
 * @returns Number of joints.
 */

/**
 * Return the original URDF file path used to construct this RobotDescription.
 * @returns Const reference to the URDF file path string.
 */

/**
 * Return the base name of the URDF file used to construct this RobotDescription.
 * @returns URDF file name (not the full path).
 */

/**
 * Return the joint index for the joint with the given name.
 * @param joint_name Name of the joint to look up.
 * @returns Index of the joint.
 */

/**
 * Return the joint indices for the provided vector of joint names in the same order.
 * @param jointNames Vector of joint names to look up.
 * @returns Vector of joint indices matching `jointNames`.
 */

/**
 * Return the joint name for the given joint index.
 * @param jointIndex Index of the joint.
 * @returns Name of the joint at `jointIndex`.
 */

/**
 * Write a human-readable representation of `robot` to `os`.
 * @param os Output stream to write to.
 * @param robot RobotDescription to format.
 * @returns Reference to the provided output stream.
 */
namespace motorium::model {

struct JointDescription {
  std::string name;
  Bounds position_bounds;
  Bounds velocity_bounds;
  Bounds torque_bounds;

  void validate() const;

  friend std::ostream& operator<<(std::ostream& os, const JointDescription& joint);
};

// A class collecting all the info used during setup of the runtime and robot.
// Since a lot of the std::unordered_map related operations use hashtable
// lookups they are not realtime this should not be used in the main loop of the
// runtime.

class RobotDescription {
 public:
  explicit RobotDescription(const std::string& urdfPath);

  explicit RobotDescription(const std::vector<JointDescription>& jointDescriptions);

  RobotDescription() = delete;

  RobotDescription(const RobotDescription&) = delete;
  RobotDescription& operator=(const RobotDescription&) = delete;
  RobotDescription(RobotDescription&&) = delete;
  RobotDescription& operator=(RobotDescription&&) = delete;

  virtual ~RobotDescription() = default;

  const std::vector<joint_index_t>& getJointIndices() const { return joint_indices_; }
  std::vector<joint_index_t> getJointIndices(std::span<const std::string> joint_names) const;
  const std::vector<std::string>& getJointNames() const { return joint_names_; }

  bool containsJoint(const std::string& jointName) const;

  const JointDescription& getJointDescription(const std::string& joint_name) const {
    return joint_name_description_map_.at(joint_name).second;
  };

  const JointDescription& getJointDescription(size_t joint_index) const { return getJointDescription(joint_names_.at(joint_index)); };

  size_t getNumJoints() const { return joint_name_description_map_.size(); }
  const std::string& getURDFPath() const { return urdf_path_; }
  const std::string getURDFName() const;

  joint_index_t getJointIndex(const std::string& joint_name) const { return joint_name_description_map_.at(joint_name).first; };
  std::vector<joint_index_t> getJointIndices(const std::vector<std::string>& jointNames) const;

  std::string getJointName(joint_index_t jointIndex) const { return joint_names_.at(jointIndex); };

  friend std::ostream& operator<<(std::ostream& os, const RobotDescription& robot);

 private:
  const std::string urdf_path_;
  absl::flat_hash_map<std::string, std::pair<joint_index_t, JointDescription>> joint_name_description_map_;

  std::vector<joint_index_t> joint_indices_;
  std::vector<std::string> joint_names_;

  // Additional sensors like IMUs can be added here later.
};

}  // namespace motorium::model