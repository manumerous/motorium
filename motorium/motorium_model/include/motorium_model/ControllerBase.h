#pragma once

#include <motorium_model/RobotJointAction.h>
#include <motorium_model/RobotState.h>

namespace motorium::model {

class ControlBase {
public:
  virtual ~ControlBase() = default;

  virtual bool ready() const = 0;

  // Computes [commands] given [joints] states and configs.
  virtual void computeJointControlAction(
      scalar_t time, const ::motorium::model::RobotState &robotState,
      ::motorium::model::RobotJointAction &robotJointAction) = 0;
};

} // namespace motorium::model