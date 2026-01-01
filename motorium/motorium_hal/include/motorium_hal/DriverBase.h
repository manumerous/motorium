#pragma once

#include <motorium_model/RobotDescription.h>
#include <motorium_model/RobotJointAction.h>
#include <motorium_model/RobotState.h>

/**
 * Abstract base class for hardware drivers in the motorium HAL.
 *
 * Provides the common interface that all HAL drivers must implement.
 */

/**
 * Construct a DriverBase and store its instance name.
 *
 * @param robotDescription Description of the robot used to configure the driver.
 * @param name Human-readable name for this driver instance.
 */

/**
 * Start the driver and prepare any hardware or resources required for operation.
 */

/**
 * Stop the driver and halt or release any hardware or resources in use.
 */

/**
 * Update the provided RobotState with the current state read from the hardware.
 *
 * @param robotState Destination object to be populated with current robot state.
 */

/**
 * Apply the given joint action to the hardware.
 *
 * @param action Joint targets or commands to be sent to the robot joints.
 */

/**
 * Reset the driver and associated hardware to a safe initial state.
 */
namespace motorium::hal {

class DriverBase {
public:
  DriverBase([[maybe_unused]] const model::RobotDescription &robotDescription,
             const std::string &name)
      : name_(name){};

  virtual void start() = 0;

  virtual void stop() = 0;

  virtual void updateRobotState(model::RobotState &robotState) = 0;

  virtual void setJointAction(const model::RobotJointAction &action) = 0;

  virtual void reset() = 0;

protected:
  std::string name_ = {};
};

} // namespace motorium::hal