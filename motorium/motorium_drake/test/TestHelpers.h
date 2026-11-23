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

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <motorium_drake/DrakeDriver.h>
#include <motorium_hal/RobotHAL.h>
#include <motorium_model/RobotJointFeedbackAction.h>
#include <motorium_model/RobotState.h>

namespace motorium::drake_sim::test {

inline constexpr double kGravity = 9.81;

inline std::string resourcePath(const std::string& file_name) {
  return std::string(TEST_RESOURCES_DIR) + "/" + file_name;
}

inline std::string pendulumUrdf() {
  return resourcePath("pendulum_fixed_base.urdf");
}

inline std::string quadrupedUrdf() {
  return resourcePath("floating_base_quadruped.urdf");
}

// Same action (setpoint + gains) for every joint.
inline std::shared_ptr<model::RobotJointFeedbackAction> makeUniformAction(
    const model::RobotDescription& description, double q_des, double kp, double kd, double feed_forward = 0.0) {
  auto action = std::make_shared<model::RobotJointFeedbackAction>(description);
  for (size_t i = 0; i < description.getNumJoints(); ++i) {
    model::JointFeedbackAction& joint_action = action->at(i);
    joint_action.q_des = q_des;
    joint_action.v_des = 0.0;
    joint_action.kp = kp;
    joint_action.kd = kd;
    joint_action.feed_forward_effort = feed_forward;
  }
  return action;
}

inline size_t stepsFor(double seconds, double dt) {
  return static_cast<size_t>(std::lround(seconds / dt));
}

// Roll and pitch of a local-to-world rotation.
inline double rollOf(const quaternion_t& q) {
  return std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
}
inline double pitchOf(const quaternion_t& q) {
  return std::asin(std::clamp(2.0 * (q.w() * q.y() - q.z() * q.x()), -1.0, 1.0));
}

}  // namespace motorium::drake_sim::test
