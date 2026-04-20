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
#include <string>
#include <vector>

#include <motorium_core/Check.h>
#include <motorium_model/RobotDescription.h>
#include <motorium_model/RobotJointFeedbackAction.h>
#include <motorium_model/RobotState.h>
#include <magic_enum/magic_enum.hpp>

namespace motorium::hal {

enum class DriverState {
  UNINITIALIZED,
  CONFIGURED,
  READY,
  RUNNING,
  STOPPING,
  FAULT,
};

// Returns a view into static storage (magic_enum::enum_name guarantees program lifetime).
inline std::string_view toString(DriverState state) {
  return magic_enum::enum_name(state);
}

bool isLegalTransition(DriverState from, DriverState to);

class DriverBase {
 public:
  // Active on all joints in the description.
  DriverBase(const model::RobotDescription& robot_description, const std::string& name)
      : name_(name), managed_joint_names_(robot_description.getJointNames()) {
    // Skipps trivial validation.
  }

  // Active on an explicit subset of joints.
  DriverBase(const model::RobotDescription& robot_description, const std::string& name, std::vector<std::string> managed_joint_names)
      : name_(name), managed_joint_names_(std::move(managed_joint_names)) {
    validateManagedJointNames(robot_description);
  }

  virtual ~DriverBase() = default;

  virtual void start() = 0;
  virtual void stop() = 0;
  void updateRobotState(model::RobotState& robot_state);
  virtual void setJointFeedbackAction(const model::RobotJointFeedbackAction& action) = 0;
  virtual void reset() = 0;

  const std::string& getName() const { return name_; }
  const std::vector<std::string>& getManagedJointNames() const { return managed_joint_names_; }
  DriverState getState() const { return state_.load(); }

 protected:
  void transitionTo(DriverState next);

  virtual void updateRobotStateImpl(model::RobotState& robot_state) = 0;

  std::string name_;
  // Todo: generalize to devices (joints, IMU's, haptic sensors, etc.)
  std::vector<std::string> managed_joint_names_;

 private:
  std::atomic<DriverState> state_{DriverState::UNINITIALIZED};

  void validateManagedJointNames(const model::RobotDescription& robot_description) const;
};

}  // namespace motorium::hal
