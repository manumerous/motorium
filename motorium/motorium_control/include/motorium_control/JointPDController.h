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

#include <motorium_control/ControllerBase.h>
#include <motorium_core/Check.h>
#include <motorium_core/Types.h>

namespace motorium::control {

struct JointPDControllerConfig {
  std::vector<std::string> joint_names;
  vector_t kp;
  vector_t kd;

  void validate() const;
};

/*
@brief
Implicit PD Controller does not compute the feedback terms itself but forwards gains and setpoints to be evaluated at the driver. This way
the feedback can happen at potentially higher loop rates. Explicit PD Controller computes the feedback terms itself and forwards the
feedback torques to the driver.
*/

template <bool IsImplicit = true>
class JointPDController : public ControllerBase {
 public:
  JointPDController(const model::RobotDescription& robot_description, const JointPDControllerConfig& config);
  ~JointPDController() override = default;

  void validateConfig() const;

  void computeJointControlAction(scalar_t time,
                                 const model::RobotState& current_state,
                                 const model::RobotState& desired_state,
                                 model::RobotJointFeedbackAction& joint_action) override;

 private:
  JointPDControllerConfig config_;
  std::vector<joint_index_t> joint_indices_;

  void computeImplicitPD(const model::RobotState& desired_state, model::RobotJointFeedbackAction& joint_action);

  void computeExplicitPD(const model::RobotState& current_state,
                         const model::RobotState& desired_state,
                         model::RobotJointFeedbackAction& joint_action);
};

// Type aliases
using ImplicitJointPDController = JointPDController<true>;
using ExplicitJointPDController = JointPDController<false>;

}  // namespace motorium::control
