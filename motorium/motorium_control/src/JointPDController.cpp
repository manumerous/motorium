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

#include <motorium_control/JointPDController.h>

#include <motorium_core/Check.h>

namespace motorium::control {

void JointPDControllerConfig::validate() const {
  MT_CHECK(!joint_names.empty()) << "[JointPDController] Configuration error: Joint names list is empty.";
  MT_CHECK(static_cast<long>(joint_names.size()) == kp.size())
      << "[JointPDController] Configuration error: Size mismatch between joint_names and kp.";
  MT_CHECK(static_cast<long>(joint_names.size()) == kd.size())
      << "[JointPDController] Configuration error: Size mismatch between joint_names and kd.";
}

template <bool IsImplicit>
JointPDController<IsImplicit>::JointPDController(const model::RobotDescription& robot_description, const JointPDControllerConfig& config)
    : ControllerBase(robot_description), config_(config), joint_indices_(robot_description.getJointIndices(config_.joint_names)) {
  config_.validate();
}

template <bool IsImplicit>
void JointPDController<IsImplicit>::computeJointControlAction([[maybe_unused]] scalar_t time,
                                                              const model::RobotState& current_state,
                                                              const model::RobotState& desired_state,
                                                              model::RobotJointFeedbackAction& joint_action) {
  if constexpr (IsImplicit) {
    computeImplicitPD(desired_state, joint_action);
  } else {
    computeExplicitPD(current_state, desired_state, joint_action);
  }
}

template <bool IsImplicit>
void JointPDController<IsImplicit>::computeImplicitPD(const model::RobotState& desired_state,
                                                      model::RobotJointFeedbackAction& joint_action) {
  for (size_t i = 0; i < joint_indices_.size(); ++i) {
    const auto joint_index = joint_indices_[i];
    if (joint_action.contains(joint_index)) {
      model::JointFeedbackAction& action = joint_action[joint_index];
      const model::JointState& desired = desired_state.getJointState(joint_index);

      action.q_des = desired.q;
      action.v_des = desired.v;
      action.feed_forward_effort = desired.effort;
      action.kp = config_.kp(i);
      action.kd = config_.kd(i);
    }
  }
}

template <bool IsImplicit>
void JointPDController<IsImplicit>::computeExplicitPD(const model::RobotState& current_state,
                                                      const model::RobotState& desired_state,
                                                      model::RobotJointFeedbackAction& joint_action) {
  for (size_t i = 0; i < joint_indices_.size(); ++i) {
    const auto joint_index = joint_indices_[i];
    if (joint_action.contains(joint_index)) {
      model::JointFeedbackAction& action = joint_action[joint_index];
      const model::JointState& desired = desired_state.getJointState(joint_index);
      const model::JointState& current = current_state.getJointState(joint_index);

      scalar_t q_error = desired.q - current.q;
      scalar_t v_error = desired.v - current.v;
      scalar_t feedback = config_.kp(i) * q_error + config_.kd(i) * v_error;

      action.q_des = desired.q;
      action.v_des = desired.v;
      action.feed_forward_effort = desired.effort + feedback;
      action.kp = 0.0;
      action.kd = 0.0;
    }
  }
}

// Explicit template instantiations
template class JointPDController<true>;
template class JointPDController<false>;

}  // namespace motorium::control
