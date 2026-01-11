#include <motorium_control/ImplicitPDController.h>

#include <motorium_core/Check.h>

namespace motorium::control {

ImplicitPDController::ImplicitPDController(const model::RobotDescription& robotDescription, const ImplicitPDControllerConfig& config)
    : ControllerBase(robotDescription), config_(config), joint_indices_(robotDescription.getJointIndices(config_.joint_names)) {
  validateConfig();
}

bool ImplicitPDController::validateConfig() const {
  MT_CHECK(!config_.joint_names.empty()) << "[ImplicitPDController] Configuration error: Joint names list is empty.";
  // vector_t::size() returns Eigen::Index (long). std::vector::size() returns size_t (ulong).
  // Cast to compare safe.
  MT_CHECK(static_cast<long>(config_.joint_names.size()) == config_.kp.size())
      << "[ImplicitPDController] Configuration error: Size mismatch between joint_names and kp.";
  MT_CHECK(static_cast<long>(config_.joint_names.size()) == config_.kd.size())
      << "[ImplicitPDController] Configuration error: Size mismatch between joint_names and kd.";
  return true;
}

void ImplicitPDController::computeJointControlAction([[maybe_unused]] scalar_t time,
                                                     [[maybe_unused]] const model::RobotState& robotState,
                                                     model::RobotJointAction& robotJointAction) {
  for (size_t i = 0; i < joint_indices_.size(); ++i) {
    const auto joint_index = joint_indices_[i];
    if (robotJointAction.contains(joint_index)) {
      model::JointAction& action = robotJointAction[joint_index];
      // Use 'i' to index into the config arrays
      action.kp = config_.kp(i);
      action.kd = config_.kd(i);
    }
  }
}

}  // namespace motorium::control
