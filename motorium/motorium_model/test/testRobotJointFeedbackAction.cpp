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
#include <cmath>

#include <motorium_core/Types.h>
#include <motorium_model/RobotJointFeedbackAction.h>

namespace {

using namespace motorium;
using namespace motorium::model;

// Measured position and velocity the collapsed action is evaluated at. Arbitrary, but nonzero so a
// dropped q_des or v_des term shows up in the torque.
constexpr scalar_t kMeasuredPosition = 0.37;
constexpr scalar_t kMeasuredVelocity = -1.25;

// The range invariant is the whole reason this is a type and not a bare scalar, so it has to hold
// for anything a caller can construct.
TEST(OutputScaler, ClampsToBounds) {
  EXPECT_DOUBLE_EQ(OutputScaler{-1.0}.getValue(), 0.0);
  EXPECT_DOUBLE_EQ(OutputScaler{5.0}.getValue(), 1.0);
  EXPECT_DOUBLE_EQ(OutputScaler{0.25}.getValue(), 0.25);
  EXPECT_DOUBLE_EQ(kFullOutputScaler.getValue(), 1.0);
}

// The whole point of scaling the gains instead of the setpoints: the joint keeps the same target,
// the controller just pushes toward it less hard.
TEST(JointFeedbackAction, ScaledLeavesSetpointsAlone) {
  const JointFeedbackAction action{.q_des = 0.85, .v_des = -0.4, .kp = 200.0, .kd = 19.8, .feed_forward_effort = 12.34};

  const JointFeedbackAction scaled = action.scaled(0.25);

  EXPECT_EQ(scaled.q_des, action.q_des);
  EXPECT_EQ(scaled.v_des, action.v_des);
  EXPECT_DOUBLE_EQ(scaled.kp, 50.0);
  EXPECT_DOUBLE_EQ(scaled.kd, 4.95);
  EXPECT_DOUBLE_EQ(scaled.feed_forward_effort, 3.085);
}

// The property callers actually rely on: scaling by `factor` scales the torque by `factor` at every
// measured state, not just at the setpoint.
TEST(JointFeedbackAction, ScaledScalesTorqueAtEveryState) {
  const JointFeedbackAction action{.q_des = 0.85, .v_des = -0.4, .kp = 200.0, .kd = 19.8, .feed_forward_effort = 12.34};
  const scalar_t torque = action.getTotalFeedbackTorque(kMeasuredPosition, kMeasuredVelocity);

  // A power of two scales every term exactly, so the two orderings agree bit for bit.
  EXPECT_EQ(action.scaled(0.5).getTotalFeedbackTorque(kMeasuredPosition, kMeasuredVelocity), 0.5 * torque);
  EXPECT_NEAR(action.scaled(0.3).getTotalFeedbackTorque(kMeasuredPosition, kMeasuredVelocity), 0.3 * torque, 1e-12);
}

// A controller scaled to zero must command exactly zero torque, with no residual from the
// setpoints.
TEST(JointFeedbackAction, ScaledByZeroCommandsNoTorque) {
  const JointFeedbackAction action{.q_des = 0.85, .v_des = -0.4, .kp = 200.0, .kd = 19.8, .feed_forward_effort = 12.34};

  EXPECT_DOUBLE_EQ(action.scaled(0.0).getTotalFeedbackTorque(kMeasuredPosition, kMeasuredVelocity), 0.0);
}

// A controller contributing only feed-forward effort (inverse dynamics) leaves kp and kd at zero.
// Collapsing that used to divide 0 by 0, and the resulting NaN q_des survived its multiplication by
// kp = 0 to poison the commanded torque.
TEST(JointAccumulaction, FeedForwardOnlyCollapsesToFiniteTorque) {
  JointAccumulaction accumulation;
  accumulation.addFeedbackAction(JointFeedbackAction{.feed_forward_effort = 12.34}, kFullOutputScaler);

  const JointFeedbackAction action = accumulation.toJointFeedbackAction();

  EXPECT_TRUE(std::isfinite(action.q_des));
  EXPECT_TRUE(std::isfinite(action.v_des));
  EXPECT_DOUBLE_EQ(action.kp, 0.0);
  EXPECT_DOUBLE_EQ(action.kd, 0.0);
  EXPECT_DOUBLE_EQ(action.getTotalFeedbackTorque(kMeasuredPosition, kMeasuredVelocity), 12.34);
}

// The gain floor must not cost accuracy: a single PD contributor has to round-trip its setpoints
// bit-identically, not merely closely.
TEST(JointAccumulaction, SingleFeedbackActionRoundTripsExactly) {
  const JointFeedbackAction original{.q_des = 0.85, .v_des = -0.4, .kp = 200.0, .kd = 19.8, .feed_forward_effort = 0.0};

  JointAccumulaction accumulation;
  accumulation.addFeedbackAction(original, kFullOutputScaler);
  const JointFeedbackAction action = accumulation.toJointFeedbackAction();

  EXPECT_EQ(action.q_des, original.q_des);
  EXPECT_EQ(action.v_des, original.v_des);
  EXPECT_EQ(action.kp, original.kp);
  EXPECT_EQ(action.kd, original.kd);
}

// Gains small enough to be physically meaningless still have to survive the floor untouched.
TEST(JointAccumulaction, TinyGainsRoundTripExactly) {
  const JointFeedbackAction original{.q_des = 1.5, .v_des = 2.5, .kp = 1e-9, .kd = 1e-9};

  JointAccumulaction accumulation;
  accumulation.addFeedbackAction(original, kFullOutputScaler);
  const JointFeedbackAction action = accumulation.toJointFeedbackAction();

  EXPECT_EQ(action.q_des, original.q_des);
  EXPECT_EQ(action.v_des, original.v_des);
}

// Stacking inverse dynamics on top of a PD law is the intended usage. The collapsed action must
// reproduce the accumulated law exactly, since only the collapsed form reaches the driver.
TEST(JointAccumulaction, CollapsePreservesStackedTorque) {
  JointAccumulaction accumulation;
  accumulation.addFeedbackAction(JointFeedbackAction{.q_des = 0.85, .v_des = -0.4, .kp = 200.0, .kd = 19.8}, kFullOutputScaler);
  accumulation.addFeedbackAction(JointFeedbackAction{.feed_forward_effort = 12.34}, kFullOutputScaler);

  const JointFeedbackAction action = accumulation.toJointFeedbackAction();

  EXPECT_DOUBLE_EQ(action.getTotalFeedbackTorque(kMeasuredPosition, kMeasuredVelocity),
                   accumulation.getTotalFeedbackTorque(kMeasuredPosition, kMeasuredVelocity));
}

// A joint no controller drives at all — the same zero-gain path, reached without any contributor.
TEST(JointAccumulaction, UndrivenJointCommandsZeroTorque) {
  const JointAccumulaction accumulation;

  const JointFeedbackAction action = accumulation.toJointFeedbackAction();

  EXPECT_DOUBLE_EQ(action.getTotalFeedbackTorque(kMeasuredPosition, kMeasuredVelocity), 0.0);
}

}  // namespace
