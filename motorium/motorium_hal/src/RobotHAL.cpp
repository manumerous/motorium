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

#include <motorium_hal/RobotHAL.h>

namespace motorium::hal {

RobotHAL::RobotHAL(const std::string& model_path, std::vector<std::shared_ptr<hal::DriverBase>> drivers)
    : robot_description_(model_path), drivers_(std::move(drivers)) {
  validateDriverCoverage();
}

const model::RobotDescription& RobotHAL::getRobotDescription() const {
  return robot_description_;
}

void RobotHAL::update(const model::RobotJointFeedbackAction& action, model::RobotState& robot_state) {
  for (const auto& driver : drivers_) {
    driver->setJointFeedbackAction(action);
  }
  for (const auto& driver : drivers_) {
    driver->updateRobotState(robot_state);
  }
}

HalState RobotHAL::getState() const {
  bool all_running = true;
  bool all_at_least_configured = true;
  for (const auto& driver : drivers_) {
    DriverState ds = driver->getState();
    if (ds == DriverState::FAULT) return HalState::FAULT;
    if (ds != DriverState::RUNNING) all_running = false;
    if (ds == DriverState::UNINITIALIZED) all_at_least_configured = false;
  }
  if (all_running) return HalState::ACTIVE;
  if (all_at_least_configured) return HalState::CONFIGURED;
  return HalState::UNCONFIGURED;
}

void RobotHAL::startDrivers() {
  for (const auto& driver : drivers_) {
    driver->start();
  }
}

void RobotHAL::stopDrivers() {
  for (const auto& driver : drivers_) {
    driver->stop();
  }
}

void RobotHAL::validateDriverCoverage() {
  std::unordered_set<std::string> managed;
  for (const auto& driver : drivers_) {
    if (!driver) {
      throw std::runtime_error("[RobotHAL] Bad Configuration: Driver list contains a null driver.");
    }
    for (const auto& joint : driver->getManagedJointNames()) {
      if (!managed.insert(joint).second) {
        throw std::runtime_error("[RobotHAL] Bad Configuration: Joint '" + joint + "' is managed by multiple drivers.");
      }
    }
  }
  for (const auto& joint : robot_description_.getJointNames()) {
    MT_CHECK(managed.find(joint) != managed.end()) << "[RobotHAL] Bad Configuration: Joint '" << joint << "' is not managed by any driver.";
  }
}

}  // namespace motorium::hal
