#pragma once

#include <motorium_core/Bounds.h>
#include <motorium_core/Types.h>

#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"

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
