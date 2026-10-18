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

#include <magic_enum/magic_enum.hpp>

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

void RobotHAL::updateState(model::RobotState& robot_state) {
  if (!ensureAllDriversRunning()) return;
  for (const auto& driver : drivers_) {
    driver->updateState(HalKey{}, robot_state);
  }
  // A driver may fault while being read, leaving its joints stale. Check again so the safety stop
  // happens before an action computed from that state can be sent.
  ensureAllDriversRunning();
}

void RobotHAL::setAction(const model::RobotJointFeedbackAction& action) {
  if (!ensureAllDriversRunning()) return;
  for (const auto& driver : drivers_) {
    driver->setAction(HalKey{}, action);  // a driver faulting in this window skips its own action
  }
}

bool RobotHAL::ensureAllDriversRunning() {
  if (safety_stopped_) return false;
  for (const auto& driver : drivers_) {
    const DriverState state = driver->getState();
    if (state != DriverState::RUNNING) {
      safetyStop("driver '" + driver->getName() + "' is " + std::string(magic_enum::enum_name(state)) + " instead of RUNNING");
      return false;
    }
  }
  return true;
}

void RobotHAL::safetyStop(const std::string& reason) {
  safety_stopped_ = true;
  std::cerr << "[RobotHAL] SAFETY STOP: " << reason << ". Stopping all drivers; call resetDrivers() to recover." << std::endl;
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
