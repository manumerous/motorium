#pragma once

#include <motorium_model/RobotDescription.h>
#include <motorium_model/RobotJointAction.h>
#include <motorium_model/RobotState.h>

namespace motorium::hal {

class DriverBase {
 public:
  DriverBase([[maybe_unused]] const model::RobotDescription& robotDescription, const std::string& name) : name_(name){};

  virtual void start() = 0;

  virtual void stop() = 0;

  virtual void updateRobotState(model::RobotState& robotState) = 0;

  virtual void setJointAction(const model::RobotJointAction& action) = 0;

  virtual void reset() = 0;

 protected:
  std::string name_ = {};
};

}  // namespace motorium::hal