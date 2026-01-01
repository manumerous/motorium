#include <motorium_model/RobotState.h>

namespace motorium::model {

/**
 * @brief Constructs a RobotState initialized from a robot description and a contact-flag count.
 *
 * Initializes the joint state map from the provided robot description, sets the internal time to 0.0,
 * resets the root and joint configurations to zero, and allocates `contact_size` contact flags all set to `true`.
 *
 * @param robot_description RobotDescription used to initialize joint state layout and metadata.
 * @param contact_size Number of contact flags to allocate and initialize to `true`.
 */
RobotState::RobotState(const RobotDescription& robot_description, size_t contact_size)
    : joint_state_map_(robot_description), time_(0.0), contact_flags_(contact_size) {
  setConfigurationToZero();

  std::fill(contact_flags_.begin(), contact_flags_.end(),
            true);  // Assume robot is in contact
}

/**
 * @brief Reset the robot state to a zeroed configuration.
 *
 * Resets the root configuration to its zero state, sets every joint's
 * position, velocity, and measured effort to 0.0, and marks all contact
 * flags as `true` (assumes the robot is in contact).
 */
void RobotState::setConfigurationToZero() {
  root_state_.setConfigurationToZero();

  for (auto& joint : joint_state_map_) {
    joint.position = 0.0;
    joint.velocity = 0.0;
    joint.measuredEffort = 0.0;
  }

  std::fill(contact_flags_.begin(), contact_flags_.end(),
            true);  // Assume robot is in contact
}

}  // namespace motorium::model