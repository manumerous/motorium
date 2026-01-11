#pragma once

#include <motorium_core/Types.h>
#include <motorium_model/RobotDescription.h>
#include <motorium_model/RobotJointAction.h>
#include <motorium_model/RobotState.h>

namespace motorium::control {

class ControllerBase {
 public:
  explicit ControllerBase([[maybe_unused]] const model::RobotDescription& robotDescription){};
  virtual ~ControllerBase() = default;

  // Computes [commands] given [joints] states and configs.
  virtual void computeJointControlAction(scalar_t time, const model::RobotState& robotState, model::RobotJointAction& robotJointAction) = 0;
};

}  // namespace motorium::control