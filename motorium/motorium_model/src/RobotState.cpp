#include <motorium_model/RobotState.h>

namespace motorium::model {

RobotState::RobotState(const RobotDescription& robot_description, size_t contact_size)
    : joint_state_map_(robot_description), time_(0.0), contact_flags_(contact_size) {
  setConfigurationToZero();

  std::fill(contact_flags_.begin(), contact_flags_.end(),
            true);  // Assume robot is in contact
}

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