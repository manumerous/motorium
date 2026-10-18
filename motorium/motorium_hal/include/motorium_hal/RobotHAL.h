/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <motorium_core/Check.h>
#include <motorium_hal/DriverBase.h>
#include <motorium_model/RobotDescription.h>
#include <motorium_model/RobotJointFeedbackAction.h>
#include <motorium_model/RobotState.h>

namespace motorium::hal {

// Unified interface to interact with real/simulated robot hardware.
// getState() returns the lowest (most degraded) DriverState across all drivers.
class RobotHAL {
 public:
  // Load description from a MuJoCo/URDF model file.
  explicit RobotHAL(const std::string& model_path);

  // Take ownership of an existing description (e.g. from tests).
  explicit RobotHAL(std::unique_ptr<model::RobotDescription> description);

  RobotHAL(const RobotHAL&) = delete;
  RobotHAL& operator=(const RobotHAL&) = delete;
  RobotHAL(RobotHAL&&) = delete;
  RobotHAL& operator=(RobotHAL&&) = delete;

  // Creates a driver of type DriverT using the HAL's description.
  // Returns a reference valid for the lifetime of this RobotHAL.
  template <typename DriverT, typename... Args>
  DriverT& addDriver(Args&&... args) {
    auto driver = std::make_unique<DriverT>(HalKey{}, *robot_description_, std::forward<Args>(args)...);
    DriverT& ref = *driver;
    drivers_.push_back(std::move(driver));
    return ref;
  }

  const model::RobotDescription& getRobotDescription() const;

  // Reflects the most degraded driver state — always current, never stale. Reports FAULT while the
  // HAL is latched in a safety stop, even though the stopped drivers themselves are READY again.
  DriverState getState() const;

  // Safety policy: startDrivers(), updateState() and setAction() require every driver to be RUNNING.
  // If any driver is not (or fails during the call), the HAL performs a safety stop: it stops all
  // drivers (each brings its hardware to rest), latches FAULT and returns the driver's error
  // unchanged. While latched, updateState() and setAction() return HalLatched without touching any
  // driver, startDrivers() aborts, and getState() reports FAULT until resetDrivers() succeeds.

  // Collects the latest state from all drivers.
  [[nodiscard]] DriverResult<> updateState(model::RobotState& robot_state);
  // Sends the action to all drivers.
  [[nodiscard]] DriverResult<> setAction(const model::RobotJointFeedbackAction& action);

  // Validates joint coverage across all registered drivers (aborting on a bad configuration), then
  // starts them. If a driver fails to start or does not reach RUNNING, performs a safety stop and
  // returns its error. Aborts while the HAL is latched in FAULT; call resetDrivers() first.
  [[nodiscard]] DriverResult<> startDrivers();
  // Stops every driver, even if some fail, and returns the first error.
  [[nodiscard]] DriverResult<> stopDrivers();
  // Resets every driver, even if some fail, and returns the first error. Clears a latched safety
  // stop only if every reset succeeded.
  [[nodiscard]] DriverResult<> resetDrivers();

 private:
  void validateDriverCoverage();

  // Succeeds if every driver is RUNNING. Otherwise performs a safety stop and returns NotRunning for
  // the first driver found not RUNNING.
  DriverResult<> ensureAllDriversRunning();
  // Stops *all* drivers (not just the one that failed) and latches FAULT. `cause` is the error that
  // triggered the stop; it is only used to report why the stop happened.
  void safetyStopAllDrivers(const DriverError& cause);

  std::atomic<bool> safety_stopped_{false};

  std::unique_ptr<model::RobotDescription> robot_description_;
  std::vector<std::unique_ptr<hal::DriverBase>> drivers_;
};

}  // namespace motorium::hal
