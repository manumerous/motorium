#pragma once

#include <motorium_core/Types.h>
#include <motorium_model/RobotDescription.h>
#include <motorium_model/RobotJointAction.h>
#include <motorium_model/RobotState.h>

namespace motorium::control {

class ControllerBase {
 public:
  explicit ControllerBase([[maybe_unused]] const model::RobotDescription& robot_description){};
  virtual ~ControllerBase() = default;

  // Computes [commands] given [joints] states and configs.
  virtual void computeJointControlAction(scalar_t time, const model::RobotState& robot_state, model::RobotJointAction& robot_joint_action) = 0;
};

}  // namespace motorium::control