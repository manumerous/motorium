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

#include <iostream>
#include <stdexcept>
#include <unordered_set>

#include <motorium_hal/DriverBase.h>

#include <magic_enum/magic_enum.hpp>

namespace motorium::hal {

DriverResult<> DriverBase::start(HalKey) {
  absl::MutexLock lock(&lifecycle_mutex_);
  const DriverState state = getState();
  if (state == DriverState::RUNNING) return {};
  MT_CHECK(state == DriverState::READY) << "Driver " << name_ << " cannot be started from state " << magic_enum::enum_name(state);

  if (DriverResult<> result = startImpl(); !result) return faultWith(result.error());
  // Only move to RUNNING if no fault was reported in the meantime.
  if (!state_.tryTransition(DriverState::READY, DriverState::RUNNING)) {
    return withName({.code = DriverErrorCode::FaultedDuringCall, .detail = "faulted while starting"});
  }
  return {};
}

DriverResult<> DriverBase::stop(HalKey) {
  absl::MutexLock lock(&lifecycle_mutex_);
  if (getState() == DriverState::FAULT) {
    // Still bring the hardware to rest; the driver stays in FAULT either way.
    if (DriverResult<> result = stopImpl(); !result) return faultWith(result.error());
    return {};
  }
  if (!state_.tryTransition(DriverState::RUNNING, DriverState::STOPPING)) return {};

  if (DriverResult<> result = stopImpl(); !result) return faultWith(result.error());
  // A fault reported while stopping must not be cleared by the transition back to READY.
  if (!state_.tryTransition(DriverState::STOPPING, DriverState::READY)) {
    return withName({.code = DriverErrorCode::FaultedDuringCall, .detail = "faulted while stopping"});
  }
  return {};
}

DriverResult<> DriverBase::reset(HalKey) {
  absl::MutexLock lock(&lifecycle_mutex_);
  const DriverState state = getState();
  MT_CHECK(state == DriverState::READY || state == DriverState::RUNNING || state == DriverState::FAULT)
      << "Driver " << name_ << " cannot be reset from state " << magic_enum::enum_name(state);

  if (DriverResult<> result = resetImpl(); !result) return faultWith(result.error());
  state_.tryTransition(DriverState::FAULT, DriverState::READY);
  return {};
}

// Not being RUNNING here is reported, not checked: a driver may report a fault from its own thread at
// any time, so it is an expected race rather than a caller bug. RobotHAL reacts to the error.
DriverResult<> DriverBase::updateState(HalKey, model::RobotState& robot_state) {
  if (getState() != DriverState::RUNNING) return withName({.code = DriverErrorCode::NotRunning, .detail = stateToString()});
  if (DriverResult<> result = updateStateImpl(robot_state); !result) return faultWith(result.error());
  if (getState() != DriverState::RUNNING) {
    return withName({.code = DriverErrorCode::FaultedDuringCall, .detail = "faulted during updateState"});
  }
  return {};
}

DriverResult<> DriverBase::setAction(HalKey, const model::RobotJointFeedbackAction& action) {
  if (getState() != DriverState::RUNNING) return withName({.code = DriverErrorCode::NotRunning, .detail = stateToString()});
  if (DriverResult<> result = setActionImpl(action); !result) return faultWith(result.error());
  if (getState() != DriverState::RUNNING) {
    return withName({.code = DriverErrorCode::FaultedDuringCall, .detail = "faulted during setAction"});
  }
  return {};
}

void DriverBase::markReady() {
  MT_CHECK(state_.tryTransition(DriverState::INITIALIZING, DriverState::READY))
      << "Driver " << name_ << " can only be marked ready while INITIALIZING, but is " << stateToString();
}

void DriverBase::reportFault(std::string_view reason) {
  if (enterFault()) std::cerr << "[" << name_ << "] FAULT: " << reason << std::endl;
}

std::unexpected<DriverError> DriverBase::withName(DriverError error) const {
  error.driver = name_;
  return std::unexpected(error);
}

std::unexpected<DriverError> DriverBase::faultWith(DriverError error) {
  std::unexpected<DriverError> named = withName(error);
  enterFault();
  std::cerr << "[" << name_ << "] FAULT: " << named.error() << std::endl;
  return named;
}

bool DriverBase::enterFault() {
  while (true) {
    const DriverState state = getState();
    if (state == DriverState::FAULT) return false;
    if (state_.tryTransition(state, DriverState::FAULT)) return true;
  }
}

void DriverBase::validateManagedJointNames(const model::RobotDescription& description) const {
  std::unordered_set<std::string> seen;
  for (const auto& joint_name : managed_joint_names_) {
    (void)description.validateName(joint_name);
    if (!seen.insert(joint_name).second) {
      throw std::invalid_argument("Duplicate joint name '" + joint_name + "' in driver '" + name_ + "'");
    }
  }
}

}  // namespace motorium::hal
