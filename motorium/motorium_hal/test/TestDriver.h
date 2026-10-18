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

#include <motorium_hal/DriverBase.h>

namespace motorium::hal {

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

  // When set, startImpl() reports a fault, mimicking hardware that fails to come up.
  bool fault_on_start_{false};
  // When set, updateStateImpl() reports a fault, mimicking a read that fails mid-tick.
  bool fault_on_update_state_{false};
  // When set, setActionImpl() reports a fault, mimicking a write that fails mid-tick.
  bool fault_on_set_action_{false};

 private:
  void startImpl() override {
    start_count_++;
    if (fault_on_start_) reportFault("start failed");
  }
  void stopImpl() override { stop_count_++; }
  void resetImpl() override { reset_count_++; }
  void updateStateImpl(motorium::model::RobotState&) override {
    update_state_count_++;
    if (fault_on_update_state_) reportFault("read failed");
  }
  void setActionImpl(const motorium::model::RobotJointFeedbackAction&) override {
    set_action_count_++;
    if (fault_on_set_action_) reportFault("write failed");
  }
};

}  // namespace motorium::hal
