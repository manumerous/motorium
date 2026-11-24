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

// Fixed-base pendulum: hinge about y, 1 kg at 0.5 m, torque limit 5 Nm.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <vector>

#include <motorium_control/JointPDController.h>

#include "TestHelpers.h"

using namespace motorium;
using namespace motorium::drake_sim;
using namespace motorium::drake_sim::test;

namespace {

constexpr double kArmLength = 0.5;
constexpr double kMass = 1.0;
constexpr double kTorqueLimit = 5.0;

class PendulumTest : public ::testing::Test {
 protected:
  void SetUp() override {
    hal_ = std::make_unique<hal::RobotHAL>(pendulumUrdf());
    state_ = std::make_unique<model::RobotState>(hal_->getRobotDescription());
    action_ = std::make_unique<model::RobotJointFeedbackAction>(hal_->getRobotDescription());
  }

  DrakeDriver& addDriver(DrakeSimConfig config) {
    config.manualStepping = true;
    return hal_->addDriver<DrakeDriver>(config);
  }

  std::shared_ptr<model::RobotState> initialState(double q) const {
    auto state = std::make_shared<model::RobotState>(hal_->getRobotDescription());
    state->setConfigurationToZero();
    state->setJointPosition(0, q);
    return state;
  }

  std::unique_ptr<hal::RobotHAL> hal_;
  std::unique_ptr<model::RobotState> state_;
  std::unique_ptr<model::RobotJointFeedbackAction> action_;
};

}  // namespace

TEST_F(PendulumTest, FreeSwingPeriodMatchesSmallAngleTheory) {
  DrakeSimConfig config;
  config.defaultJointDamping = 0.0;
  config.initStatePtr = initialState(0.05);
  DrakeDriver& sim = addDriver(config);
  hal_->startDrivers();

  // Times at which q crosses zero going downwards; consecutive ones are one period apart.
  std::vector<double> crossings;
  double previous_q = 0.05;
  for (size_t step = 0; step < stepsFor(5.0, config.dt) && crossings.size() < 3; ++step) {
    ASSERT_TRUE(sim.advance());
    hal_->updateState(*state_);
    const double q = state_->getJointPosition(0);
    if (previous_q > 0.0 && q <= 0.0) {
      crossings.push_back(state_->getTime());
    }
    previous_q = q;
  }
  ASSERT_EQ(crossings.size(), 3u);

  const double expected_period = 2.0 * std::numbers::pi * std::sqrt(kArmLength / kGravity);
  EXPECT_NEAR(crossings[1] - crossings[0], expected_period, 0.01 * expected_period);
  EXPECT_NEAR(crossings[2] - crossings[1], expected_period, 0.01 * expected_period);
  hal_->stopDrivers();
}

TEST_F(PendulumTest, ImplicitPdControllerHoldsSetpoint) {
  DrakeSimConfig config;
  DrakeDriver& sim = addDriver(config);

  const double kp = 200.0;
  control::ImplicitJointPDController controller(hal_->getRobotDescription(), {{"hinge"}, {kp}, {20.0}});
  model::RobotState desired(hal_->getRobotDescription());
  desired.setConfigurationToZero();
  desired.setJointPosition(0, 0.4);

  hal_->startDrivers();
  for (size_t step = 0; step < stepsFor(3.0, config.dt); ++step) {
    hal_->updateState(*state_);
    controller.computeJointControlAction(state_->getTime(), *state_, desired, *action_);
    hal_->setAction(*action_);
    ASSERT_TRUE(sim.advance());
  }
  hal_->updateState(*state_);

  // Steady state: kp * (q_des - q) balances gravity m g l sin(q), so q sits slightly below 0.4.
  double q_expected = 0.4;
  for (int i = 0; i < 50; ++i) {
    q_expected = 0.4 - kMass * kGravity * kArmLength * std::sin(q_expected) / kp;
  }
  EXPECT_NEAR(state_->getJointPosition(0), q_expected, 1e-3);
  EXPECT_NEAR(state_->getJointVelocity(0), 0.0, 1e-3);
  EXPECT_NEAR(state_->getJointEffort(0), kMass * kGravity * kArmLength * std::sin(q_expected), 0.02);
  hal_->stopDrivers();
}

TEST_F(PendulumTest, TorqueIsClampedToTheJointTorqueBounds) {
  DrakeSimConfig config;
  DrakeDriver& sim = addDriver(config);
  hal_->startDrivers();

  for (const double q_des : {1.0, -1.0}) {
    action_->at(0) = {.q_des = q_des, .v_des = 0.0, .kp = 1000.0, .kd = 0.0, .feed_forward_effort = 0.0};
    hal_->setAction(*action_);
    ASSERT_TRUE(sim.advance());
    hal_->updateState(*state_);
    EXPECT_DOUBLE_EQ(state_->getJointEffort(0), std::copysign(kTorqueLimit, q_des));
  }
  hal_->stopDrivers();
}

TEST_F(PendulumTest, NonFiniteActionAppliesZeroTorqueWithoutFaulting) {
  DrakeSimConfig config;
  DrakeDriver& sim = addDriver(config);
  hal_->startDrivers();

  action_->at(0) = {.q_des = 0.0, .v_des = 0.0, .kp = 0.0, .kd = 0.0, .feed_forward_effort = std::numeric_limits<double>::quiet_NaN()};
  hal_->setAction(*action_);
  ASSERT_TRUE(sim.advance(10));
  hal_->updateState(*state_);
  EXPECT_DOUBLE_EQ(state_->getJointEffort(0), 0.0);
  EXPECT_TRUE(std::isfinite(state_->getJointPosition(0)));
  EXPECT_EQ(hal_->getState(), hal::DriverState::RUNNING);
  hal_->stopDrivers();
}

TEST_F(PendulumTest, JointDampingOpposesMotion) {
  DrakeSimConfig config;
  config.defaultJointDamping = 2.0;
  config.initStatePtr = initialState(0.5);
  DrakeDriver& sim = addDriver(config);
  hal_->startDrivers();

  ASSERT_TRUE(sim.advance(stepsFor(10.0, config.dt)));
  hal_->updateState(*state_);
  // Undamped it would still swing with amplitude 0.5 rad; damped it has settled.
  EXPECT_NEAR(state_->getJointPosition(0), 0.0, 0.01);
  hal_->stopDrivers();
}

TEST_F(PendulumTest, InitialActionHoldsUntilTheFirstSetAction) {
  DrakeSimConfig config;
  config.defaultJointDamping = 0.0;  // otherwise the released pendulum creeps down too slowly to see
  config.initStatePtr = initialState(0.5);
  config.initialActionPtr = makeUniformAction(hal_->getRobotDescription(), 0.5, 400.0, 40.0,
                                              kMass * kGravity * kArmLength * std::sin(0.5));  // gravity compensation
  DrakeDriver& sim = addDriver(config);
  hal_->startDrivers();

  // No setAction() yet: the initial action holds the pendulum at 0.5 rad.
  ASSERT_TRUE(sim.advance(stepsFor(1.0, config.dt)));
  hal_->updateState(*state_);
  EXPECT_NEAR(state_->getJointPosition(0), 0.5, 1e-3);

  // The first setAction() replaces it: zero torque, so the pendulum swings away.
  action_->at(0) = {};
  hal_->setAction(*action_);
  ASSERT_TRUE(sim.advance(stepsFor(0.2, config.dt)));
  hal_->updateState(*state_);
  EXPECT_DOUBLE_EQ(state_->getJointEffort(0), 0.0);
  EXPECT_LT(state_->getJointPosition(0), 0.45);

  // After a reset the initial action applies again.
  hal_->resetDrivers();
  ASSERT_TRUE(sim.advance(stepsFor(1.0, config.dt)));
  hal_->updateState(*state_);
  EXPECT_NEAR(state_->getJointPosition(0), 0.5, 1e-3);
  hal_->stopDrivers();
}
