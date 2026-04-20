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
#include <string>
#include <unordered_set>
#include <vector>

#include <motorium_core/Check.h>
#include <motorium_hal/DriverBase.h>
#include <motorium_model/RobotDescription.h>
#include <motorium_model/RobotJointFeedbackAction.h>
#include <motorium_model/RobotState.h>
#include <magic_enum/magic_enum.hpp>

namespace motorium::hal {

// Unified interface to interact with real/simulated robot hardware.

enum class HalState {
  UNCONFIGURED,
  CONFIGURED,
  READY,
  STOPPING,
  ACTIVE,
  FAULT,
};

// Returns a view into static storage (magic_enum::enum_name guarantees program lifetime).
inline std::string_view toString(HalState state) {
  return magic_enum::enum_name(state);
}

class RobotHAL {
 public:
  RobotHAL(const std::string& model_path, std::vector<std::shared_ptr<hal::DriverBase>> drivers);

  RobotHAL(const RobotHAL&) = delete;
  RobotHAL& operator=(const RobotHAL&) = delete;
  RobotHAL(RobotHAL&&) = delete;
  RobotHAL& operator=(RobotHAL&&) = delete;

  const model::RobotDescription& getRobotDescription() const;

  void update(const model::RobotJointFeedbackAction& action, model::RobotState& robot_state);

  // Computed on the fly to prevent stale state from diverging with driver states.
  HalState getState() const;

  void startDrivers();
  void stopDrivers();

 private:
  void validateDriverCoverage();

  const model::RobotDescription robot_description_;
  std::vector<std::shared_ptr<hal::DriverBase>> drivers_;
};

}  // namespace motorium::hal
