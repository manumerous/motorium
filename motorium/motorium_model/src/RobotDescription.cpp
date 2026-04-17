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

#include <urdfdom/urdf_parser/urdf_parser.h>
#include <pugixml.hpp>

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

    // Per-joint actuator info extracted from <actuator> children.
    struct ActuatorInfo {
      double force_min = -std::numeric_limits<double>::infinity();
      double force_max = std::numeric_limits<double>::infinity();
      double ctrl_min = -std::numeric_limits<double>::infinity();
      double ctrl_max = std::numeric_limits<double>::infinity();
      bool has_forcerange = false;
      bool has_ctrlrange = false;
      bool ctrl_limited = false;
      bool is_direct_torque = false;  // true for <motor> or gain="1" actuators
    };

    std::unordered_map<std::string, ActuatorInfo> actuator_info_map;
    for (const pugi::xml_node& actuator : doc.child("mujoco").child("actuator").children()) {
      const std::string joint_name = actuator.attribute("joint").as_string();
      if (joint_name.empty()) continue;

      ActuatorInfo info;

      // forcerange
      const std::string force_range = actuator.attribute("forcerange").as_string();
      if (!force_range.empty()) {
        std::istringstream ss(force_range);
        ss >> info.force_min >> info.force_max;
        info.has_forcerange = true;
      }

      // ctrlrange — only relevant when ctrllimited="true"
      const std::string ctrl_range = actuator.attribute("ctrlrange").as_string();
      const std::string ctrl_limited_str = actuator.attribute("ctrllimited").as_string();
      info.ctrl_limited = (ctrl_limited_str == "true");
      if (!ctrl_range.empty()) {
        std::istringstream ss(ctrl_range);
        ss >> info.ctrl_min >> info.ctrl_max;
        info.has_ctrlrange = true;
      }

      // Determine if this is a direct-torque actuator.
      // <motor> elements are direct-torque by definition.
      // General actuators with gain="1" (or default gain) also qualify.
      const std::string element_name = actuator.name();
      const double gain = actuator.attribute("gain").as_double(0.0);
      info.is_direct_torque = (element_name == "motor") || (gain == 1.0);

      actuator_info_map[joint_name] = info;
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

      // Compute torque bounds as the intersection (tightest envelope) of all
      // applicable sources:
      //   1. actuatorfrcrange on the <joint> element
      //   2. forcerange on the matching <actuator> element
      //   3. ctrlrange on the matching <actuator> element (only when
      //      ctrllimited="true" and the actuator is a direct-torque motor)
      // This is to ensure that there is no case where the read out torque bounds are stricter in
      // sim than what will be applied on the hardware.

      double t_min = -std::numeric_limits<double>::infinity();
      double t_max = std::numeric_limits<double>::infinity();

      auto narrow = [&](double lo, double hi) {
        t_min = std::max(t_min, lo);
        t_max = std::min(t_max, hi);
      };

      // Source 1: actuatorfrcrange on <joint>
      const std::string frc_range = joint_node.attribute("actuatorfrcrange").as_string();
      if (!frc_range.empty()) {
        double frc_min = 0.0, frc_max = 0.0;
        std::istringstream fss(frc_range);
        fss >> frc_min >> frc_max;
        narrow(frc_min, frc_max);
      }

      // Source 2 & 3: from the matching actuator
      const auto it = actuator_info_map.find(joint_name);
      if (it != actuator_info_map.end()) {
        const ActuatorInfo& info = it->second;

        if (info.has_forcerange) {
          narrow(info.force_min, info.force_max);
        }

        if (info.has_ctrlrange && info.ctrl_limited && info.is_direct_torque) {
          narrow(info.ctrl_min, info.ctrl_max);
        }
      }

      // Only override defaults if at least one source provided bounds.
      if (t_min > -std::numeric_limits<double>::infinity() || t_max < std::numeric_limits<double>::infinity()) {
        joint_desc.torque_bounds.min = t_min;
        joint_desc.torque_bounds.max = t_max;
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