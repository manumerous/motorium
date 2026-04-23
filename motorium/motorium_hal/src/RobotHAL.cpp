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
    : motorium::core::StateMachine<HalState>(HalState::UNCONFIGURED), robot_description_(model_path), drivers_(std::move(drivers)) {
  validateDriverCoverage();
}

const model::RobotDescription& RobotHAL::getRobotDescription() const {
  return robot_description_;
}

bool RobotHAL::isLegalTransition(HalState from, HalState to) const {
  if (to == HalState::FAULT) return from != HalState::FAULT;
  switch (from) {
    case HalState::UNCONFIGURED:
      return to == HalState::CONFIGURED;
    case HalState::CONFIGURED:
      return to == HalState::READY;
    case HalState::READY:
      return to == HalState::ACTIVE;
    case HalState::ACTIVE:
      return to == HalState::STOPPING;
    case HalState::STOPPING:
      return to == HalState::READY;
    case HalState::FAULT:
      return to == HalState::CONFIGURED;
    default:
      return false;
  }
}

void RobotHAL::update(const model::RobotJointFeedbackAction& action, model::RobotState& robot_state) {
  for (const auto& driver : drivers_) {
    driver->update(action, robot_state);
  }
  if (getState() != HalState::FAULT) {
    for (const auto& driver : drivers_) {
      if (driver->getState() == DriverState::FAULT) {
        requestTransitionTo(HalState::FAULT);
        break;
      }
    }
  }
}

void RobotHAL::startDrivers() {
  for (const auto& driver : drivers_) {
    driver->start();
  }
  requestTransitionTo(HalState::CONFIGURED);
  requestTransitionTo(HalState::READY);
  requestTransitionTo(HalState::ACTIVE);
}

void RobotHAL::stopDrivers() {
  for (const auto& driver : drivers_) {
    driver->stop();
  }
  requestTransitionTo(HalState::STOPPING);
}

void RobotHAL::validateDriverCoverage() {
  std::unordered_set<std::string> managed;
  for (const auto& driver : drivers_) {
    MT_CHECK(driver != nullptr) << "[RobotHAL] Bad Configuration: Driver list contains a null driver.";
    for (const auto& joint : driver->getManagedJointNames()) {
      MT_CHECK(managed.insert(joint).second) << "[RobotHAL] Bad Configuration: Joint '" << joint << "' is managed by multiple drivers.";
    }
  }
  for (const auto& joint : robot_description_.getJointNames()) {
    MT_CHECK(managed.find(joint) != managed.end()) << "[RobotHAL] Bad Configuration: Joint '" << joint << "' is not managed by any driver.";
  }
}

}  // namespace motorium::hal
