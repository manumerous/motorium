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

#include <gtest/gtest.h>
#include <motorium_hal/DriverBase.h>

#include <optional>

namespace motorium::hal {

// gtest predicate for DriverResult: EXPECT_TRUE(IsOk(result)) prints the error if the call failed.
inline ::testing::AssertionResult IsOk(const DriverResult<>& result) {
  if (result) return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << "call failed: " << result.error();
}

// Minimal concrete driver for testing DriverBase. Counts lifecycle calls and exposes the protected
// hooks a real driver would use internally.
class TestDriver : public DriverBase {
 public:
  // With mark_ready = false the driver stays INITIALIZING until finishInitialization() is called,
  // mimicking a driver with asynchronous initialization.
  TestDriver(HalKey key, const model::RobotDescription& description, std::string name, bool mark_ready = true)
      : DriverBase(key, description, std::move(name)) {
    if (mark_ready) markReady();
  }

  TestDriver(HalKey key,
             const model::RobotDescription& description,
             std::string name,
             std::vector<std::string> managed_joints,
             bool mark_ready = true)
      : DriverBase(key, description, std::move(name), std::move(managed_joints)) {
    if (mark_ready) markReady();
  }

  void finishInitialization() { markReady(); }
  void triggerFault(std::string_view reason = "test fault") { reportFault(reason); }

  int start_count_{0};
  int stop_count_{0};
  int reset_count_{0};
  int update_state_count_{0};
  int set_action_count_{0};

  // When set, the hook returns an error with this code (and fail_joint_, if set), mimicking a call
  // that fails in the driver itself.
  std::optional<DriverErrorCode> fail_start_with_;
  std::optional<DriverErrorCode> fail_stop_with_;
  std::optional<DriverErrorCode> fail_reset_with_;
  std::optional<DriverErrorCode> fail_update_state_with_;
  std::optional<DriverErrorCode> fail_set_action_with_;
  std::optional<joint_index_t> fail_joint_;

  // When set, the hook reports a fault and then returns successfully, mimicking a fault reported from
  // the driver's own thread while the call is running.
  bool fault_during_start_{false};
  bool fault_during_update_state_{false};
  bool fault_during_set_action_{false};

 private:
  DriverResult<> result(const std::optional<DriverErrorCode>& fail_with) const {
    if (fail_with) return driverError(*fail_with, "test failure", fail_joint_);
    return {};
  }

  DriverResult<> startImpl() override {
    start_count_++;
    if (fault_during_start_) reportFault("start failed");
    return result(fail_start_with_);
  }
  DriverResult<> stopImpl() override {
    stop_count_++;
    return result(fail_stop_with_);
  }
  DriverResult<> resetImpl() override {
    reset_count_++;
    return result(fail_reset_with_);
  }
  DriverResult<> updateStateImpl(motorium::model::RobotState&) override {
    update_state_count_++;
    if (fault_during_update_state_) reportFault("read failed");
    return result(fail_update_state_with_);
  }
  DriverResult<> setActionImpl(const motorium::model::RobotJointFeedbackAction&) override {
    set_action_count_++;
    if (fault_during_set_action_) reportFault("write failed");
    return result(fail_set_action_with_);
  }
};

}  // namespace motorium::hal
