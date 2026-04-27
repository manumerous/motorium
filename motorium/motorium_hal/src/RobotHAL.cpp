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
  return std::accumulate(drivers_.begin(), drivers_.end(), DriverState::RUNNING,
                         [](DriverState worst, const std::unique_ptr<DriverBase>& driver) { return std::min(worst, driver->getState()); });
}

void RobotHAL::update(const model::RobotJointFeedbackAction& action, model::RobotState& robot_state) {
  for (const auto& driver : drivers_) {
    if (driver->getState() == DriverState::RUNNING) {
      driver->update(HalKey{}, action, robot_state);
    }
  }
}

void RobotHAL::startDrivers() {
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
