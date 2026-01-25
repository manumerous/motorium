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

#include "motorium_model/RobotDescription.h"
// #include <pugixml.hpp>

#include <urdfdom/urdf_parser/urdf_parser.h>
// #include <urdf_parser/urdf_parser.h>
#include <filesystem>
#include <fstream>
// #include <stdexcept>

namespace motorium::model {

// Constructor
RobotDescription::RobotDescription(const std::string& urdfPath) : urdf_path_(urdfPath) {
  // Validate URDF file exists
  if (!std::filesystem::exists(urdfPath)) {
    throw std::runtime_error("URDF file not found: " + urdfPath);
  }

  // Read URDF file content
  std::ifstream urdfFile(urdfPath);
  std::string urdfContent((std::istreambuf_iterator<char>(urdfFile)), std::istreambuf_iterator<char>());
  urdfFile.close();

  // Parse URDF
  urdf::ModelInterfaceSharedPtr urdfModel = urdf::parseURDF(urdfContent);
  if (!urdfModel) {
    throw std::runtime_error("Failed to parse URDF file: " + urdfPath);
  }
  joint_indices_.reserve(urdfModel->joints_.size());
  joint_names_.reserve(urdfModel->joints_.size());

  // Process all joints from the URDF
  int32_t joint_id = 0;
  for (const auto& jointPair : urdfModel->joints_) {
    const std::string& joint_name = jointPair.first;
    const urdf::JointSharedPtr& joint = jointPair.second;

    // Skip fixed joints, continuous joints, and other non-controllable
    // types
    if (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::PRISMATIC) {
      continue;
    }

    // Create joint description
    JointDescription joint_desc;
    joint_desc.name = joint_name;

    // Get joint limits
    if (joint->limits) {
      joint_desc.position_bounds.min = joint->limits->lower;
      joint_desc.position_bounds.max = joint->limits->upper;
      joint_desc.velocity_bounds.min = -joint->limits->velocity;
      joint_desc.velocity_bounds.max = joint->limits->velocity;
      joint_desc.torque_bounds.min = -joint->limits->effort;
      joint_desc.torque_bounds.max = joint->limits->effort;  // Fixed this line
    }

    // Add joint to maps
    joint_name_description_map_[joint_name] = std::make_pair(joint_id, joint_desc);
    joint_indices_.push_back(joint_id);
    joint_names_.push_back(joint_name);
    joint_id++;
  }

  // Validate that we found at least one joint
  if (joint_name_description_map_.empty()) {
    throw std::runtime_error("No valid joints found in URDF: " + urdf_path_);
  }
}

RobotDescription::RobotDescription(const std::vector<JointDescription>& joint_descriptions) : urdf_path_("") {
  // Reserve space for efficiency
  joint_indices_.reserve(joint_descriptions.size());
  joint_names_.reserve(joint_descriptions.size());

  // Process each joint description
  size_t joint_id = 0;
  for (const auto& joint_desc : joint_descriptions) {
    // Validate the joint description
    joint_desc.validate();

    // Check for duplicate joint names
    if (joint_name_description_map_.contains(joint_desc.name)) {
      throw std::invalid_argument("Duplicate joint name: " + joint_desc.name);
    }

    // Add to maps
    joint_name_description_map_[joint_desc.name] = std::make_pair(joint_id, joint_desc);

    // Add to vectors
    joint_indices_.push_back(joint_id);
    joint_names_.push_back(joint_desc.name);
    joint_id++;
  }
}

bool RobotDescription::containsJoint(const std::string& jointName) const {
 
  return joint_name_description_map_.contains(jointName);
}

std::vector<joint_index_t> RobotDescription::getJointIndices(std::span<const std::string> joint_names) const {
  std::vector<joint_index_t> indices;
  indices.reserve(joint_names.size());

  for (const std::string& joint_name : joint_names) {
    indices.push_back(joint_name_description_map_.at(validateName(joint_name)).first);
  }

  return indices;
}

const std::string RobotDescription::getURDFName() const {
  std::size_t lastSlashPos = urdf_path_.find_last_of("/");
  if (lastSlashPos == std::string::npos) {
    return urdf_path_;
  }

  // Return string after the last '/'
  return urdf_path_.substr(lastSlashPos + 1);
}

std::ostream& operator<<(std::ostream& os, const JointDescription& joint) {
  os << "JointDescription { " << "name: " << joint.name << ", Position " << joint.position_bounds << ", Velocity " << joint.velocity_bounds
     << ", Torque " << joint.torque_bounds << " }";
  return os;
}

std::ostream& operator<<(std::ostream& os, const RobotDescription& robot) {
  os << "RobotDescription {" << std::endl;
  os << "Generated from URDF: " << robot.getURDFPath() << std::endl;
  os << " Joint names and descriptions:" << std::endl;
  for (const auto& joint : robot.joint_name_description_map_) {
    os << "  {" << ", id " << joint.second.first << ": " << joint.second.second << " }" << std::endl;
  }

  os << "}";
  return os;
}

void JointDescription::validate() const {
  auto validate = [](const Bounds& bounds) {
    if (bounds.min > bounds.max) {
      throw std::invalid_argument("min > max");
    }
  };
  validate(position_bounds);
  validate(velocity_bounds);
  validate(torque_bounds);
}

}  // namespace motorium::model