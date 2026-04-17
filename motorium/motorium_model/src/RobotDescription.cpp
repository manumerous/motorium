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

#include <pugixml.hpp>
#include <urdfdom/urdf_parser/urdf_parser.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace motorium::model {

RobotDescription::RobotDescription(const std::string& model_path) : model_path_(model_path) {
  if (!std::filesystem::exists(model_path)) {
    throw std::runtime_error("Model file not found: " + model_path);
  }

  const std::string ext = std::filesystem::path(model_path).extension().string();

  // Read a small prefix to validate that file content matches the extension.
  std::ifstream preview_file(model_path);
  std::string preview(4096, '\0');
  preview_file.read(preview.data(), 512);
  preview.resize(preview_file.gcount());
  preview_file.close();

  if (ext == ".urdf") {
    if (preview.find("<robot") == std::string::npos) {
      throw std::runtime_error("File has .urdf extension but does not contain a <robot> element: " + model_path);
    }

    std::ifstream urdf_file(model_path);
    const std::string urdf_content((std::istreambuf_iterator<char>(urdf_file)), std::istreambuf_iterator<char>());

    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(urdf_content);
    if (!urdf_model) {
      throw std::runtime_error("Failed to parse URDF: " + model_path);
    }

    joint_indices_.reserve(urdf_model->joints_.size());
    joint_names_.reserve(urdf_model->joints_.size());

    int32_t joint_id = 0;
    for (const auto& joint_pair : urdf_model->joints_) {
      const std::string& joint_name = joint_pair.first;
      const urdf::JointSharedPtr& joint = joint_pair.second;

      if (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::PRISMATIC) {
        continue;
      }

      JointDescription joint_desc;
      joint_desc.name = joint_name;

      if (joint->limits) {
        joint_desc.position_bounds.min = joint->limits->lower;
        joint_desc.position_bounds.max = joint->limits->upper;
        joint_desc.velocity_bounds.min = -joint->limits->velocity;
        joint_desc.velocity_bounds.max = joint->limits->velocity;
        joint_desc.torque_bounds.min = -joint->limits->effort;
        joint_desc.torque_bounds.max = joint->limits->effort;
      }

      joint_name_description_map_[joint_name] = std::make_pair(joint_id, joint_desc);
      joint_indices_.push_back(joint_id);
      joint_names_.push_back(joint_name);
      joint_id++;
    }

  } else if (ext == ".xml") {
    if (preview.find("<mujoco") == std::string::npos) {
      throw std::runtime_error("File has .xml extension but does not contain a <mujoco> element: " + model_path);
    }

    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(model_path.c_str());
    if (!result) {
      throw std::runtime_error("Failed to parse MuJoCo XML: " + model_path + ". Error: " + result.description());
    }

    // Build joint name → torque limits from <actuator> children.
    // Matches any actuator element that has a joint= and forcerange= attribute.
    std::unordered_map<std::string, std::pair<double, double>> joint_torque_limits;
    for (const pugi::xml_node& actuator : doc.child("mujoco").child("actuator").children()) {
      const std::string joint_name = actuator.attribute("joint").as_string();
      const std::string force_range = actuator.attribute("forcerange").as_string();
      if (joint_name.empty() || force_range.empty()) continue;

      double fmin = 0.0, fmax = 0.0;
      std::istringstream ss(force_range);
      ss >> fmin >> fmax;
      joint_torque_limits[joint_name] = {fmin, fmax};
    }

    // Walk all <joint> nodes anywhere inside <worldbody>, skip free/ball joints.
    int32_t joint_id = 0;
    for (const pugi::xpath_node& xpath_joint : doc.child("mujoco").child("worldbody").select_nodes(".//joint")) {
      const pugi::xml_node& joint_node = xpath_joint.node();
      const std::string type = joint_node.attribute("type").as_string("hinge");
      if (type == "free" || type == "ball") continue;

      const std::string joint_name = joint_node.attribute("name").as_string();
      if (joint_name.empty()) continue;

      JointDescription joint_desc;
      joint_desc.name = joint_name;

      const std::string range = joint_node.attribute("range").as_string();
      if (!range.empty()) {
        std::istringstream ss(range);
        ss >> joint_desc.position_bounds.min >> joint_desc.position_bounds.max;
      }

      // Joint-level actuatorfrcrange takes priority over actuator forcerange.
      const std::string frc_range = joint_node.attribute("actuatorfrcrange").as_string();
      if (!frc_range.empty()) {
        std::istringstream fss(frc_range);
        fss >> joint_desc.torque_bounds.min >> joint_desc.torque_bounds.max;
      } else {
        const auto it = joint_torque_limits.find(joint_name);
        if (it != joint_torque_limits.end()) {
          joint_desc.torque_bounds.min = it->second.first;
          joint_desc.torque_bounds.max = it->second.second;
        }
      }

      joint_name_description_map_[joint_name] = std::make_pair(joint_id, joint_desc);
      joint_indices_.push_back(joint_id);
      joint_names_.push_back(joint_name);
      joint_id++;
    }

  } else {
    throw std::runtime_error("Unsupported model file format '" + ext + "': expected .urdf or .xml");
  }

  if (joint_name_description_map_.empty()) {
    throw std::runtime_error("No valid joints found in model: " + model_path);
  }
}

RobotDescription::RobotDescription(const std::vector<JointDescription>& joint_descriptions) : model_path_("") {
  joint_indices_.reserve(joint_descriptions.size());
  joint_names_.reserve(joint_descriptions.size());

  size_t joint_id = 0;
  for (const auto& joint_desc : joint_descriptions) {
    joint_desc.validate();

    if (joint_name_description_map_.contains(joint_desc.name)) {
      throw std::invalid_argument("Duplicate joint name: " + joint_desc.name);
    }

    joint_name_description_map_[joint_desc.name] = std::make_pair(joint_id, joint_desc);
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

const std::string RobotDescription::getModelName() const {
  const std::size_t lastSlashPos = model_path_.find_last_of("/");
  if (lastSlashPos == std::string::npos) {
    return model_path_;
  }
  return model_path_.substr(lastSlashPos + 1);
}

std::ostream& operator<<(std::ostream& os, const JointDescription& joint) {
  os << "JointDescription { " << "name: " << joint.name << ", Position " << joint.position_bounds << ", Velocity " << joint.velocity_bounds
     << ", Torque " << joint.torque_bounds << " }";
  return os;
}

std::ostream& operator<<(std::ostream& os, const RobotDescription& robot) {
  os << "RobotDescription {" << std::endl;
  os << "Generated from: " << (robot.getModelPath().empty() ? "(manual joint descriptions)" : robot.getModelPath()) << std::endl;
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