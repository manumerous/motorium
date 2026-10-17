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

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numbers>
#include <pugixml.hpp>
#include <sstream>
#include <unordered_map>
#include <utility>

namespace motorium::model {

namespace {

constexpr scalar_t kInf = std::numeric_limits<scalar_t>::infinity();

// Parses a MuJoCo "lo hi" range attribute. Returns false if the attribute is absent or malformed.
bool parseRange(const pugi::xml_node& node, const char* attribute, scalar_t& lo, scalar_t& hi) {
  const std::string value = node.attribute(attribute).as_string();
  if (value.empty()) {
    return false;
  }
  std::istringstream ss(value);
  scalar_t parsed_lo = 0.0;
  scalar_t parsed_hi = 0.0;
  if (!(ss >> parsed_lo >> parsed_hi)) {
    return false;
  }
  lo = parsed_lo;
  hi = parsed_hi;
  return true;
}

// MuJoCo's "limited" attributes are "true", "false" or "auto" (the default), where "auto" means
// limited whenever the corresponding range is given.
bool isLimited(const pugi::xml_node& node, const char* attribute, bool has_range) {
  const std::string value = node.attribute(attribute).as_string("auto");
  return value == "true" || (value == "auto" && has_range);
}

}  // namespace

RobotDescription::RobotDescription(const std::string& model_path) : model_path_(model_path) {
  if (!std::filesystem::exists(model_path)) {
    throw std::runtime_error("Model file not found: " + model_path);
  }

  const std::string ext = std::filesystem::path(model_path).extension().string();
  if (ext == ".urdf") {
    initializeFromUrdf(model_path);
  } else if (ext == ".xml") {
    initializeFromMujocoXml(model_path);
  } else {
    throw std::runtime_error("Unsupported model file format '" + ext + "': expected .urdf or .xml (MuJoCo).");
  }

  // Validate that we found at least one joint
  if (joint_name_description_map_.empty()) {
    throw std::runtime_error("No valid joints found in model: " + model_path);
  }
}

RobotDescription::RobotDescription(const std::vector<JointDescription>& joint_descriptions, std::string model_path)
    : model_path_(std::move(model_path)) {
  joint_indices_.reserve(joint_descriptions.size());
  joint_names_.reserve(joint_descriptions.size());

  for (const auto& joint_desc : joint_descriptions) {
    addJoint(joint_desc);
  }
}

void RobotDescription::addJoint(const JointDescription& joint_desc) {
  // Validate the joint description
  joint_desc.validate();

  // Check for duplicate joint names
  if (joint_name_description_map_.contains(joint_desc.name)) {
    throw std::invalid_argument("Duplicate joint name: " + joint_desc.name);
  }

  const joint_index_t joint_id = joint_indices_.size();
  joint_name_description_map_.insert_or_assign(joint_desc.name, std::make_pair(joint_id, joint_desc));
  joint_indices_.push_back(joint_id);
  joint_names_.push_back(joint_desc.name);
}

void RobotDescription::initializeFromUrdf(const std::string& urdf_path) {
  // Read URDF file content
  std::ifstream urdf_file(urdf_path);
  std::string urdf_content((std::istreambuf_iterator<char>(urdf_file)), std::istreambuf_iterator<char>());
  urdf_file.close();

  // Parse URDF
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(urdf_content);
  if (!urdf_model) {
    throw std::runtime_error("Failed to parse URDF file: " + urdf_path);
  }
  joint_indices_.reserve(urdf_model->joints_.size());
  joint_names_.reserve(urdf_model->joints_.size());

  // Process all joints from the URDF
  for (const auto& [joint_name, joint] : urdf_model->joints_) {
    // Skip fixed joints, continuous joints, and other non-controllable
    // types
    if (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::PRISMATIC) {
      continue;
    }

    JointDescription joint_desc;
    joint_desc.name = joint_name;

    if (joint->limits) {
      joint_desc.position_bounds = Bounds(joint->limits->lower, joint->limits->upper);
      joint_desc.velocity_bounds = Bounds(-joint->limits->velocity, joint->limits->velocity);
      joint_desc.torque_bounds = Bounds(-joint->limits->effort, joint->limits->effort);
    }

    addJoint(joint_desc);
  }
}

void RobotDescription::initializeFromMujocoXml(const std::string& mujoco_xml_path) {
  pugi::xml_document doc;
  const pugi::xml_parse_result result = doc.load_file(mujoco_xml_path.c_str());
  if (!result) {
    throw std::runtime_error("Failed to parse MuJoCo XML: " + mujoco_xml_path + ". Error: " + result.description());
  }

  const pugi::xml_node mujoco = doc.child("mujoco");
  if (!mujoco) {
    throw std::runtime_error("File has .xml extension but does not contain a <mujoco> root element: " + mujoco_xml_path);
  }

  // MJCF specifies hinge ranges in degrees unless <compiler angle="radian"> is set.
  const bool angles_in_degrees = std::string(mujoco.child("compiler").attribute("angle").as_string("degree")) != "radian";

  // Per-joint actuator info extracted from <actuator> children.
  struct ActuatorInfo {
    scalar_t force_min = -kInf;
    scalar_t force_max = kInf;
    scalar_t ctrl_min = -kInf;
    scalar_t ctrl_max = kInf;
    bool force_limited = false;
    bool ctrl_limited = false;
    bool is_direct_torque = false;  // true if ctrl maps 1:1 to joint torque
  };

  std::unordered_map<std::string, ActuatorInfo> actuator_info_map;
  for (const pugi::xml_node& actuator : mujoco.child("actuator").children()) {
    const std::string joint_name = actuator.attribute("joint").as_string();
    if (joint_name.empty()) {
      continue;
    }

    ActuatorInfo info;
    const bool has_forcerange = parseRange(actuator, "forcerange", info.force_min, info.force_max);
    info.force_limited = isLimited(actuator, "forcelimited", has_forcerange) && has_forcerange;
    const bool has_ctrlrange = parseRange(actuator, "ctrlrange", info.ctrl_min, info.ctrl_max);
    info.ctrl_limited = isLimited(actuator, "ctrllimited", has_ctrlrange) && has_ctrlrange;

    // A <motor> is direct torque by definition. A <general> actuator is direct torque with its
    // defaults (fixed gain of 1, no bias); position/velocity servos and others are not.
    const std::string element_name = actuator.name();
    if (element_name == "motor") {
      info.is_direct_torque = true;
    } else if (element_name == "general") {
      const std::string gain_type = actuator.attribute("gaintype").as_string("fixed");
      const std::string bias_type = actuator.attribute("biastype").as_string("none");
      std::istringstream gain_ss(actuator.attribute("gainprm").as_string("1"));
      scalar_t gain = 1.0;
      gain_ss >> gain;
      info.is_direct_torque = gain_type == "fixed" && bias_type == "none" && gain == 1.0;
    }

    actuator_info_map[joint_name] = info;
  }

  // Walk all <joint> nodes anywhere inside <worldbody>, skip free/ball joints.
  for (const pugi::xpath_node& xpath_joint : mujoco.child("worldbody").select_nodes(".//joint")) {
    const pugi::xml_node joint_node = xpath_joint.node();
    const std::string type = joint_node.attribute("type").as_string("hinge");
    if (type != "hinge" && type != "slide") {
      continue;
    }

    const std::string joint_name = joint_node.attribute("name").as_string();
    if (joint_name.empty()) {
      continue;
    }

    scalar_t pos_min = -kInf;
    scalar_t pos_max = kInf;
    if (parseRange(joint_node, "range", pos_min, pos_max) && type == "hinge" && angles_in_degrees) {
      pos_min *= std::numbers::pi / 180.0;
      pos_max *= std::numbers::pi / 180.0;
    }

    // Compute torque bounds as the intersection (tightest envelope) of all applicable sources:
    //   1. actuatorfrcrange on the <joint> element
    //   2. forcerange on the matching <actuator> element
    //   3. ctrlrange on the matching <actuator> element (only when it is limited and the
    //      actuator is a direct-torque motor)
    // This ensures that the torque bounds read out here are never looser than what the simulation
    // applies.
    scalar_t t_min = -kInf;
    scalar_t t_max = kInf;
    auto narrow = [&](scalar_t lo, scalar_t hi) {
      t_min = std::max(t_min, lo);
      t_max = std::min(t_max, hi);
    };

    scalar_t frc_min = 0.0;
    scalar_t frc_max = 0.0;
    if (parseRange(joint_node, "actuatorfrcrange", frc_min, frc_max)) {
      narrow(frc_min, frc_max);
    }

    const auto it = actuator_info_map.find(joint_name);
    if (it != actuator_info_map.end()) {
      const ActuatorInfo& info = it->second;
      if (info.force_limited) {
        narrow(info.force_min, info.force_max);
      }
      if (info.ctrl_limited && info.is_direct_torque) {
        narrow(info.ctrl_min, info.ctrl_max);
      }
    }

    JointDescription joint_desc;
    joint_desc.name = joint_name;
    joint_desc.position_bounds = Bounds(pos_min, pos_max);
    joint_desc.velocity_bounds = Bounds(-kInf, kInf);  // velocity bounds are not encoded in MuJoCo XML
    joint_desc.torque_bounds = Bounds(t_min, t_max);

    addJoint(joint_desc);
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

std::string RobotDescription::getModelName() const {
  const std::size_t last_slash_pos = model_path_.find_last_of('/');
  if (last_slash_pos == std::string::npos) {
    return model_path_;
  }

  // Return string after the last '/'
  return model_path_.substr(last_slash_pos + 1);
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
  // Iterate in index order (the hash map order is unspecified).
  for (const std::string& name : robot.getJointNames()) {
    os << "  { id " << robot.getJointIndex(name) << ": " << robot.getJointDescription(name) << " }" << std::endl;
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
