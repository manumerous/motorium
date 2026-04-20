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

#include <motorium_core/Check.h>
#include <motorium_hal/DriverBase.h>
#include <motorium_model/RobotJointFeedbackAction.h>
#include <motorium_model/RobotState.h>

#include "motorium_model/RobotDescription.h"

namespace motorium::hal {

// Unified interface to interact with real/simulated robot hardware.

enum class HalState {
  UNCONFIGURED,
  CONFIGURED,
  ACTIVE,
  FAULT,
};

inline std::string_view toString(HalState state) {
  return magic_enum::enum_name(state);
}

class RobotHAL {
 public:
  RobotHAL(const std::string& model_path, std::vector<std::shared_ptr<hal::DriverBase>> drivers)
      : robot_description_(model_path), drivers_(std::move(drivers)) {
    std::unordered_set<std::string> managed;
    for (const auto& driver : drivers_) {
      if (!driver) {
        throw std::runtime_error("[RobotHAL] Bad Configuration: Driver list contains a null driver.");
      }
      for (const auto& joint : driver->getManagedJointNames()) {
        if (!managed.insert(joint).second) {
          throw std::runtime_error("[RobotHAL] Bad Configuration: Joint '" + joint +
                                   "' is managed by multiple drivers.");
        }
      }
    }
    for (const auto& joint : robot_description_.getJointNames()) {
      MT_CHECK(managed.find(joint) != managed.end())
          << "[RobotHAL] Bad Configuration: Joint '" << joint << "' is not managed by any driver.";
    }
  }

  RobotHAL(const RobotHAL&) = delete;
  RobotHAL& operator=(const RobotHAL&) = delete;
  RobotHAL(RobotHAL&&) = delete;
  RobotHAL& operator=(RobotHAL&&) = delete;

  const model::RobotDescription& getRobotDescription() const { return robot_description_; }

  void update(const model::RobotJointFeedbackAction& action, model::RobotState& robot_state) {
    for (const auto& driver : drivers_) {
      driver->setJointFeedbackAction(action);
    }

    for (const auto& driver : drivers_) {
      driver->updateRobotState(robot_state);
    }
  }

  // Computed on the fly for now to prevent mismatch between driver states and hal state.
  // This should be reevaluated depending o how often and where this function is called.
  // Currently computing the state triggers a cache sync for every driver.
  HalState getState() const {
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

  void startDrivers() {
    for (const auto& driver : drivers_) {
      driver->start();
    }
  }

  void stopDrivers() {
    for (const auto& driver : drivers_) {
      driver->stop();
    }
  }

 private:
  const model::RobotDescription robot_description_;
  std::vector<std::shared_ptr<hal::DriverBase>> drivers_;
};

}  // namespace motorium::hal
