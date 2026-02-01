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

#include <gtest/gtest.h>
#include <motorium_control/ImplicitPDController.h>
#include <motorium_model/RobotDescription.h>
#include <motorium_model/RobotJointFeedbackAction.h>
#include <motorium_model/RobotState.h>

using namespace motorium;
using namespace motorium::control;
using namespace motorium::model;

class ImplicitPDControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a dummy robot description with 2 joints
    JointDescription j1;
    j1.name = "joint1";
    JointDescription j2;
    j2.name = "joint2";

    std::vector<JointDescription> joints = {j1, j2};
    robot_description_ = std::make_unique<RobotDescription>(joints);
  }

  std::unique_ptr<RobotDescription> robot_description_;
};

TEST_F(ImplicitPDControllerTest, InitializationValidConfig) {
  ImplicitPDControllerConfig config;
  config.joint_names = {"joint1"};
  vector_t kp(1);
  kp << 10.0;
  vector_t kd(1);
  kd << 1.0;
  config.kp = kp;
  config.kd = kd;

  ImplicitPDController controller(*robot_description_, config);
}

TEST_F(ImplicitPDControllerTest, InitializationInvalidConfigSize) {
  ImplicitPDControllerConfig config;
  config.joint_names = {"joint1"};
  vector_t kp(1);
  kp << 10.0;
  config.kp = kp;
  config.kd = vector_t(0);  // Mismatch

  EXPECT_DEATH(ImplicitPDController(*robot_description_, config), "Size mismatch");
}

TEST_F(ImplicitPDControllerTest, InitializationMissingJoint) {
  ImplicitPDControllerConfig config;
  config.joint_names = {"missing_joint"};
  vector_t kp(1);
  kp << 10.0;
  vector_t kd(1);
  kd << 1.0;
  config.kp = kp;
  config.kd = kd;

  EXPECT_THROW(ImplicitPDController(*robot_description_, config), std::out_of_range);
}

TEST_F(ImplicitPDControllerTest, ComputeAction) {
  ImplicitPDControllerConfig config;
  config.joint_names = {"joint1", "joint2"};
  vector_t kp(2);
  kp << 10.0, 20.0;
  vector_t kd(2);
  kd << 1.0, 2.0;
  config.kp = kp;
  config.kd = kd;

  ImplicitPDController controller(*robot_description_, config);

  RobotState state(*robot_description_);
  RobotJointFeedbackAction action(*robot_description_);

  controller.computeJointControlAction(0.0, state, state, action);

  // Check joint1
  joint_index_t idx1 = robot_description_->getJointIndex("joint1");
  EXPECT_DOUBLE_EQ(action[idx1].kp, 10.0);
  EXPECT_DOUBLE_EQ(action[idx1].kd, 1.0);

  // Check joint2
  joint_index_t idx2 = robot_description_->getJointIndex("joint2");
  EXPECT_DOUBLE_EQ(action[idx2].kp, 20.0);
  EXPECT_DOUBLE_EQ(action[idx2].kd, 2.0);
}

TEST_F(ImplicitPDControllerTest, PartialControl) {
  // Control only joint2
  ImplicitPDControllerConfig config;
  config.joint_names = {"joint2"};
  vector_t kp(1);
  kp << 5.0;
  vector_t kd(1);
  kd << 0.5;
  config.kp = kp;
  config.kd = kd;

  ImplicitPDController controller(*robot_description_, config);

  RobotState state(*robot_description_);
  RobotJointFeedbackAction action(*robot_description_);

  // Initialize action with zeros
  joint_index_t idx1 = robot_description_->getJointIndex("joint1");
  joint_index_t idx2 = robot_description_->getJointIndex("joint2");
  action[idx1].kp = 0.0;
  action[idx2].kp = 0.0;

  controller.computeJointControlAction(0.0, state, state, action);

  EXPECT_DOUBLE_EQ(action[idx1].kp, 0.0);  // Should be untouched
  EXPECT_DOUBLE_EQ(action[idx2].kp, 5.0);
  EXPECT_DOUBLE_EQ(action[idx2].kd, 0.5);
}
