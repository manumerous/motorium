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

#include <algorithm>
#include <iostream>
#include <numeric>
#include <unordered_set>

#include <motorium_hal/RobotHAL.h>

namespace motorium::hal {

RobotHAL::RobotHAL(const std::string& model_path) : robot_description_(std::make_unique<model::RobotDescription>(model_path)) {}

RobotHAL::RobotHAL(std::unique_ptr<model::RobotDescription> description) : robot_description_(std::move(description)) {}

const model::RobotDescription& RobotHAL::getRobotDescription() const {
  return *robot_description_;
}

DriverState RobotHAL::getState() const {
  if (safety_stopped_) return DriverState::FAULT;
  return std::accumulate(drivers_.begin(), drivers_.end(), DriverState::RUNNING,
                         [](DriverState worst, const std::unique_ptr<DriverBase>& driver) { return std::min(worst, driver->getState()); });
}

// Each driver checks its own state before and after the call, so its *Impl() never runs on a
// non-running driver. The first error triggers the safety stop, before any remaining driver is read
// or commanded. Drivers earlier in the loop may already have been read or commanded on that tick;
// they are brought to rest by the stop right after.
DriverResult<> RobotHAL::updateState(model::RobotState& robot_state) {
  if (safety_stopped_) return driverError(DriverErrorCode::HalLatched, "call resetDrivers() to recover");
  for (const auto& driver : drivers_) {
    if (DriverResult<> result = driver->updateState(HalKey{}, robot_state); !result) {
      safetyStopAllDrivers(result.error());
      return result;
    }
  }
  return {};
}

DriverResult<> RobotHAL::setAction(const model::RobotJointFeedbackAction& action) {
  if (safety_stopped_) return driverError(DriverErrorCode::HalLatched, "call resetDrivers() to recover");
  for (const auto& driver : drivers_) {
    if (DriverResult<> result = driver->setAction(HalKey{}, action); !result) {
      safetyStopAllDrivers(result.error());
      return result;
    }
  }
  return {};
}

DriverResult<> RobotHAL::ensureAllDriversRunning() {
  for (const auto& driver : drivers_) {
    if (driver->getState() != DriverState::RUNNING) {
      const DriverError error{.code = DriverErrorCode::NotRunning, .detail = driver->stateToString(), .driver = driver->getName()};
      safetyStopAllDrivers(error);
      return std::unexpected(error);
    }
  }
  return {};
}

void RobotHAL::safetyStopAllDrivers(const DriverError& cause) {
  safety_stopped_ = true;
  std::cerr << "[RobotHAL] SAFETY STOP: " << cause << ". Stopping all drivers; call resetDrivers() to recover." << std::endl;
  // Drivers that fail to stop move to FAULT and log the failure themselves; there is nothing more the
  // HAL can do about it here, and the HAL is latched in FAULT either way.
  (void)stopDrivers();
}

DriverResult<> RobotHAL::startDrivers() {
  MT_CHECK(!safety_stopped_) << "[RobotHAL] Cannot start drivers while latched in FAULT after a safety stop; call resetDrivers() first.";
  validateDriverCoverage();
  for (const auto& driver : drivers_) {
    if (DriverResult<> result = driver->start(HalKey{}); !result) {
      safetyStopAllDrivers(result.error());
      return result;
    }
  }
  // A driver may also have faulted from its own thread after it started; it must not leave the others
  // running on their own.
  return ensureAllDriversRunning();
}

DriverResult<> RobotHAL::stopDrivers() {
  DriverResult<> first_result;
  for (const auto& driver : drivers_) {
    DriverResult<> result = driver->stop(HalKey{});
    if (!result && first_result) first_result = result;
  }
  return first_result;
}

DriverResult<> RobotHAL::resetDrivers() {
  DriverResult<> first_result;
  for (const auto& driver : drivers_) {
    DriverResult<> result = driver->reset(HalKey{});
    if (!result && first_result) first_result = result;
  }
  // A driver whose reset failed is back in FAULT, so the HAL stays latched.
  if (first_result) safety_stopped_ = false;
  return first_result;
}

void RobotHAL::validateDriverCoverage() {
  std::unordered_set<std::string> managed;
  for (const auto& driver : drivers_) {
    MT_CHECK(driver != nullptr) << "[RobotHAL] Bad Configuration: Driver list contains a null driver.";
    for (const auto& joint : driver->getManagedJointNames()) {
      MT_CHECK(robot_description_->containsJoint(joint)) << "[RobotHAL] Bad Configuration: Joint '" << joint << "' managed by driver '"
                                                         << driver->getName() << "' is not present in the robot description.";
      MT_CHECK(managed.insert(joint).second) << "[RobotHAL] Bad Configuration: Joint '" << joint << "' is managed by multiple drivers.";
    }
  }
  for (const auto& joint : robot_description_->getJointNames()) {
    MT_CHECK(managed.find(joint) != managed.end()) << "[RobotHAL] Bad Configuration: Joint '" << joint << "' is not managed by any driver.";
  }
}

}  // namespace motorium::hal
