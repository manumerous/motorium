#include "motorium_model/RobotDescription.h"
// #include <pugixml.hpp>

#include <urdfdom/urdf_parser/urdf_parser.h>
// #include <urdf_parser/urdf_parser.h>
#include <filesystem>
#include <fstream>
// #include <stdexcept>

namespace motorium::model {

/**
 * @brief Construct a RobotDescription by loading and processing a URDF file.
 *
 * Parses the URDF at the given filesystem path, extracts controllable joints
 * (REVOLUTE and PRISMATIC), and populates internal joint indices, names, and
 * per-joint bounds (position, velocity, torque). Joint indices start at zero
 * and are assigned in the order encountered in the URDF.
 *
 * @param urdfPath Filesystem path to the URDF file to load.
 * @throws std::runtime_error if the URDF file does not exist, if parsing fails,
 *         or if no valid controllable joints are found in the URDF.
 */
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

/**
 * @brief Constructs a RobotDescription from an explicit list of joint descriptions.
 *
 * Initializes the robot description with an empty URDF path and populates internal
 * joint indices, names, and the name→(id, JointDescription) map from the provided
 * vector in input order.
 *
 * @param joint_descriptions Vector of joint descriptions to populate the RobotDescription.
 *
 * @throws std::invalid_argument If any JointDescription::validate() fails or if a duplicate
 *         joint name is encountered in the input.
 */
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

/**
 * @brief Check whether a joint with the given name exists in the description.
 *
 * @returns `true` if a joint with `jointName` is present, `false` otherwise.
 */
bool RobotDescription::containsJoint(const std::string& jointName) const {
  return joint_name_description_map_.contains(jointName);
}

/**
 * @brief Map a sequence of joint names to their corresponding joint indices.
 *
 * @param joint_names Span of joint names to resolve; order is preserved.
 * @return std::vector<joint_index_t> Vector of joint indices corresponding to each input name.
 * @throws std::out_of_range if any provided joint name is not present in the robot description.
 */
std::vector<joint_index_t> RobotDescription::getJointIndices(std::span<const std::string> joint_names) const {
  std::vector<joint_index_t> indices;
  indices.reserve(joint_names.size());

  for (const std::string& joint_name : joint_names) {
    indices.push_back(joint_name_description_map_.at(joint_name).first);
  }

  return indices;
}

/**
 * @brief Get the URDF file's basename from the stored path.
 *
 * @return std::string The portion of `urdf_path_` after the last '/' character.
 * If `urdf_path_` contains no '/', returns `urdf_path_` unchanged.
 */
const std::string RobotDescription::getURDFName() const {
  std::size_t lastSlashPos = urdf_path_.find_last_of("/");
  if (lastSlashPos == std::string::npos) {
    return urdf_path_;
  }

  // Return string after the last '/'
  return urdf_path_.substr(lastSlashPos + 1);
}

/**
 * @brief Writes a human-readable representation of a JointDescription to an output stream.
 *
 * @param os Stream to write into.
 * @param joint JointDescription to format.
 * @return std::ostream& Reference to the same output stream.
 */
std::ostream& operator<<(std::ostream& os, const JointDescription& joint) {
  os << "JointDescription { " << "name: " << joint.name << ", Position " << joint.position_bounds << ", Velocity " << joint.velocity_bounds
     << ", Torque " << joint.torque_bounds << " }";
  return os;
}

/**
 * @brief Format a RobotDescription into a human-readable textual representation and write it to an output stream.
 *
 * The emitted text includes the source URDF path followed by each joint's id and corresponding JointDescription.
 *
 * @return std::ostream& Reference to the output stream `os` after writing the formatted RobotDescription.
 */
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

/**
 * @brief Validates that the joint's position, velocity, and torque bounds are well-formed.
 *
 * Checks that for each bounds set (position_bounds, velocity_bounds, torque_bounds) the
 * minimum value is less than or equal to the maximum value.
 *
 * @throws std::invalid_argument if any bounds have min > max.
 */
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