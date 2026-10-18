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

#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <absl/synchronization/mutex.h>
#include <motorium_core/StateMachine.h>
#include <motorium_hal/DriverError.h>
#include <motorium_model/RobotDescription.h>
#include <motorium_model/RobotJointFeedbackAction.h>
#include <motorium_model/RobotState.h>

namespace motorium::hal {

class RobotHAL;

// Passkey: only RobotHAL can construct it, so only RobotHAL can create drivers.
struct HalKey {
  friend class RobotHAL;

 private:
  HalKey() = default;
};

// Lifecycle of a driver. Transitions are owned by DriverBase (see start/stop/reset below); a
// concrete driver can only signal markReady() once initialized and reportFault() on errors.
//
//   INITIALIZING --markReady()--> READY --start()--> RUNNING --stop()--> STOPPING --> READY
//   any state except FAULT --reportFault()--> FAULT --reset()--> READY
//
// IMPORTANT: The declaration order is load-bearing. States are ordered from most degraded to most
// operational, and RobotHAL::getState() reports the minimum (std::min) across all drivers. Reordering
// these values silently changes what the HAL reports.
// ToDO update to logic required by HW
enum class DriverState {
  FAULT,
  STOPPING,
  INITIALIZING,
  READY,
  RUNNING,
};

inline bool isLegalDriverTransition(DriverState from, DriverState to) {
  if (to == DriverState::FAULT) return from != DriverState::FAULT;
  switch (from) {
    case DriverState::INITIALIZING:
      return to == DriverState::READY;
    case DriverState::READY:
      return to == DriverState::RUNNING;
    case DriverState::RUNNING:
      return to == DriverState::STOPPING;
    case DriverState::STOPPING:
      return to == DriverState::READY;
    case DriverState::FAULT:
      return to == DriverState::READY;
    default:
      return false;
  }
}

// Base class for all drivers. It owns the driver's lifecycle state: start(), stop() and reset() check
// and perform the state transitions, while subclasses only implement the hardware/sim work in the
// *Impl() hooks. The state machine is a private member, so nothing else can change the state.
class DriverBase {
 public:
  // Manages all joints in the HAL's description.
  DriverBase(HalKey, const model::RobotDescription& description, std::string name)
      : name_(std::move(name)), managed_joint_names_(description.getJointNames()) {}

  // Manages an explicit subset; validates names exist in description and are unique.
  DriverBase(HalKey, const model::RobotDescription& description, std::string name, std::vector<std::string> managed_joint_names)
      : name_(std::move(name)), managed_joint_names_(std::move(managed_joint_names)) {
    validateManagedJointNames(description);
  }

  virtual ~DriverBase() = default;

  DriverState getState() const { return state_.getState(); }
  std::string_view stateToString() const { return state_.stateToString(); }

  // Callable only by RobotHAL — HalKey enforces this at compile time.
  //
  // All of these return a DriverResult. If an *Impl() hook returns an error, the driver moves to
  // FAULT and the error is returned with the driver's name filled in. Errors from the driver's own
  // thread (reportFault()) during a call are returned as FaultedDuringCall.

  // READY -> RUNNING. A no-op when already RUNNING; aborts from any other state, since starting a
  // driver that is not ready is a bug in the caller.
  [[nodiscard]] DriverResult<> start(HalKey);

  // RUNNING -> STOPPING -> READY. Stopping must always be safe to call during shutdown, so it never
  // aborts: it is a no-op when not running, and on a FAULTed driver it still calls stopImpl() to
  // bring the hardware to rest, but stays in FAULT.
  [[nodiscard]] DriverResult<> stop(HalKey);

  // Allowed from READY, RUNNING and FAULT (FAULT -> READY); aborts from INITIALIZING and STOPPING.
  [[nodiscard]] DriverResult<> reset(HalKey);

  // Writes the latest measured state of the managed joints into robot_state. Returns NotRunning
  // without calling updateStateImpl() unless the driver is RUNNING.
  [[nodiscard]] DriverResult<> updateState(HalKey, model::RobotState& robot_state);

  // Forwards the commanded action for the managed joints to the hardware/sim. Returns NotRunning
  // without calling setActionImpl() unless the driver is RUNNING.
  [[nodiscard]] DriverResult<> setAction(HalKey, const model::RobotJointFeedbackAction& action);

  const std::string& getName() const { return name_; }
  const std::vector<std::string>& getManagedJointNames() const { return managed_joint_names_; }

 protected:
  // INITIALIZING -> READY. Call once the driver is ready to be started, e.g. at the end of the
  // constructor or when asynchronous initialization completes.
  void markReady();

  // Moves the driver to FAULT (no-op if it already is). Safe to call from any thread. Use this for
  // faults detected outside a call, e.g. on the driver's own communication thread; inside an *Impl()
  // hook, return an error instead.
  void reportFault(std::string_view reason);

  // Hardware/sim work only; DriverBase performs all state checks and transitions around these.
  // Return an error (see driverError()) if the work failed; DriverBase then moves the driver to
  // FAULT. Leave DriverError::driver empty, DriverBase fills it in.
  virtual DriverResult<> startImpl() = 0;
  virtual DriverResult<> stopImpl() = 0;
  virtual DriverResult<> resetImpl() = 0;
  virtual DriverResult<> updateStateImpl(model::RobotState& robot_state) = 0;
  virtual DriverResult<> setActionImpl(const model::RobotJointFeedbackAction& action) = 0;

  const std::string name_;
  const std::vector<std::string> managed_joint_names_;

 private:
  void validateManagedJointNames(const model::RobotDescription& description) const;

  // Returns `error` as a failed result with this driver's name filled in.
  std::unexpected<DriverError> withName(DriverError error) const;
  // Moves the driver to FAULT, logs `error` and returns it with this driver's name filled in.
  std::unexpected<DriverError> faultWith(DriverError error);
  // Moves the driver to FAULT unless it already is. Returns whether this call caused the transition.
  bool enterFault();

  core::StateMachine<DriverState, isLegalDriverTransition> state_{DriverState::INITIALIZING};

  // Serializes start/stop/reset so concurrent lifecycle calls cannot interleave.
  absl::Mutex lifecycle_mutex_;
};

}  // namespace motorium::hal
