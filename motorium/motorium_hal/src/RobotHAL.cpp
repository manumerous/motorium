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

// Both calls first check all drivers, so a known fault means nothing is read or sent at all. A driver
// can still fault during the loop (from its own thread, or inside its *Impl()); its call then returns
// false and the safety stop happens immediately, before any remaining driver is read or commanded.
// A driver that faults after its own read has returned is caught by setAction()'s check, before any
// action computed from the stale state is sent.
void RobotHAL::updateState(model::RobotState& robot_state) {
  if (!ensureAllDriversRunning()) return;
  for (const auto& driver : drivers_) {
    if (!driver->updateState(HalKey{}, robot_state)) {
      safetyStop(*driver);
      return;
    }
  }
}

void RobotHAL::setAction(const model::RobotJointFeedbackAction& action) {
  if (!ensureAllDriversRunning()) return;
  for (const auto& driver : drivers_) {
    if (!driver->setAction(HalKey{}, action)) {
      safetyStop(*driver);
      return;
    }
  }
}

bool RobotHAL::ensureAllDriversRunning() {
  if (safety_stopped_) return false;
  for (const auto& driver : drivers_) {
    if (driver->getState() != DriverState::RUNNING) {
      safetyStop(*driver);
      return false;
    }
  }
  return true;
}

void RobotHAL::safetyStop(const DriverBase& cause) {
  safety_stopped_ = true;
  std::cerr << "[RobotHAL] SAFETY STOP: driver '" << cause.getName() << "' is " << cause.stateToString()
            << " instead of RUNNING. Stopping all drivers; call resetDrivers() to recover." << std::endl;
  stopDrivers();
}

void RobotHAL::startDrivers() {
  MT_CHECK(!safety_stopped_) << "[RobotHAL] Cannot start drivers while latched in FAULT after a safety stop; call resetDrivers() first.";
  validateDriverCoverage();
  for (const auto& driver : drivers_) {
    driver->start(HalKey{});
  }
}

void RobotHAL::stopDrivers() {
  for (const auto& driver : drivers_) {
    driver->stop(HalKey{});
  }
}

void RobotHAL::resetDrivers() {
  for (const auto& driver : drivers_) {
    driver->reset(HalKey{});
  }
  safety_stopped_ = false;
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
