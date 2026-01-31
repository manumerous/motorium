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
    return joint_name_description_map_.at(validateName(joint_name)).second;
  };

  const JointDescription& getJointDescription(size_t joint_index) const {
    return getJointDescription(joint_names_.at(validateIndex(joint_index)));
  };

  size_t getNumJoints() const { return joint_name_description_map_.size(); }
  const std::string& getURDFPath() const { return urdf_path_; }
  const std::string getURDFName() const;

  joint_index_t getJointIndex(const std::string& joint_name) const {
    return joint_name_description_map_.at(validateName(joint_name)).first;
  };

  std::string getJointName(joint_index_t jointIndex) const { return joint_names_.at(validateIndex(jointIndex)); };

  friend std::ostream& operator<<(std::ostream& os, const RobotDescription& robot);

 private:
  inline joint_index_t validateIndex(joint_index_t index) const {
    MT_CHECK(index < joint_indices_.size()) << "Joint index " << index << " out of bounds of RobotDescription.";
    return index;
  }

  inline const std::string& validateName(const std::string& name) const {
    MT_CHECK(containsJoint(name)) << "Joint " << name << " not found in RobotDescription.";
    return name;
  }
  const std::string urdf_path_;
  absl::flat_hash_map<std::string, std::pair<joint_index_t, JointDescription>> joint_name_description_map_;

  std::vector<joint_index_t> joint_indices_;
  std::vector<std::string> joint_names_;

  // Additional sensors like IMUs can be added here later.
};

}  // namespace motorium::model
