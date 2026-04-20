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

#include <memory>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include <motorium_hal/DriverBase.h>
#include <motorium_model/RobotJointFeedbackAction.h>
#include <motorium_model/RobotState.h>

#include "motorium_model/RobotDescription.h"

namespace motorium::hal {

// Unified interface to interact with real/simulated robot hardware.

class RobotHAL {
 public:
  RobotHAL(const std::string& model_path, std::vector<std::shared_ptr<hal::DriverBase>> drivers)
      : robot_description_(model_path), drivers_(std::move(drivers)) {
    std::unordered_set<std::string> managed;
    for (const auto& driver : drivers_) {
      for (const auto& joint : driver->getManagedJointNames()) {
        managed.insert(joint);
      }
    }
    for (const auto& joint : robot_description_.getJointNames()) {
      if (managed.find(joint) == managed.end()) {
        throw std::runtime_error("[RobotHAL] Bad Configuration: Joint '" + joint + "' is not managed by any driver.");
      }
    }
  }

  const model::RobotDescription& getRobotDescription() const { return robot_description_; }

  void updateRobotState(model::RobotState& robot_state) const {
    for (const auto& driver : drivers_) {
      driver->updateRobotState(robot_state);
    }
  }

  void setJointFeedbackAction(const model::RobotJointFeedbackAction& action) {
    for (const auto& driver : drivers_) {
      driver->setJointFeedbackAction(action);
    }
  }

  void startDrivers() {
    for (const auto& driver : drivers_) {
      driver->start();
      // Todo check correct initialization of drivers and update robot state machine
    }
  }

  void stopDrivers() {
    for (const auto& driver : drivers_) {
      driver->stop();
    }
  }

 private:
  // Todo add state machine that describes the state/health of the full robot.
  const model::RobotDescription robot_description_;
  std::vector<std::shared_ptr<hal::DriverBase>> drivers_;
};

}  // namespace motorium::hal
