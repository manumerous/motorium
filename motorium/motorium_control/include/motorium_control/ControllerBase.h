#pragma once

#include <motorium_model/RobotDescription.h>
#include <motorium_model/RobotJointAction.h>
#include <motorium_model/RobotState.h>

/**
 * @brief Abstract base interface for controllers that compute joint control actions.
 *
 * Implementations determine readiness and produce joint commands based on time and the
 * current robot state.
 */
 
/**
 * @brief Construct a ControlBase.
 * @param robotDescription Robot model description; implementations may ignore this.
 */

/**
 * @brief Indicate whether the controller is ready to compute control actions.
 * @returns `true` if the controller is ready to compute actions, `false` otherwise.
 */

/**
 * @brief Compute joint control commands from the current time and robot state.
 * @param time Current time used by the controller to compute commands.
 * @param robotState Current measured or estimated robot state.
 * @param robotJointAction Output container to be populated with computed joint commands.
 */
namespace motorium::control {

class ControlBase {
 public:
  ControlBase([[maybe_unused]] const model::RobotDescription& robotDescription) = default;
  virtual ~ControlBase() = default;

  virtual bool ready() const = 0;

  // Computes [commands] given [joints] states and configs.
  virtual void computeJointControlAction(scalar_t time, const model::RobotState& robotState, model::RobotJointAction& robotJointAction) = 0;
};

}  // namespace motorium::control