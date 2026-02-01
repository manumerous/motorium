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

#include <motorium_control/ImplicitPDController.h>

#include <motorium_core/Check.h>

namespace motorium::control {

ImplicitPDController::ImplicitPDController(const model::RobotDescription& robot_description, const ImplicitPDControllerConfig& config)
    : ControllerBase(robot_description), config_(config), joint_indices_(robot_description.getJointIndices(config_.joint_names)) {
  validateConfig();
}

void ImplicitPDController::validateConfig() const {
  MT_CHECK(!config_.joint_names.empty()) << "[ImplicitPDController] Configuration error: Joint names list is empty.";
  // vector_t::size() returns Eigen::Index (long). std::vector::size() returns size_t (ulong).
  // Cast to compare safe.
  MT_CHECK(static_cast<long>(config_.joint_names.size()) == config_.kp.size())
      << "[ImplicitPDController] Configuration error: Size mismatch between joint_names and kp.";
  MT_CHECK(static_cast<long>(config_.joint_names.size()) == config_.kd.size())
      << "[ImplicitPDController] Configuration error: Size mismatch between joint_names and kd.";
}

void ImplicitPDController::computeJointControlAction([[maybe_unused]] scalar_t time,
                                                     [[maybe_unused]] const model::RobotState& robot_state,
                                                     model::RobotJointAction& robot_joint_action) {
  for (size_t i = 0; i < joint_indices_.size(); ++i) {
    const auto joint_index = joint_indices_[i];
    if (robot_joint_action.contains(joint_index)) {
      model::JointAction& action = robot_joint_action[joint_index];
      // Use 'i' to index into the config arrays
      action.kp = config_.kp(i);
      action.kd = config_.kd(i);
    }
  }
}

}  // namespace motorium::control
