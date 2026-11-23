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

// Floating-base quadruped table: base box + four hip-pitch legs with spherical feet. Standing height
// of the base center with straight legs: 0.27 m.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <motorium_control/JointPDController.h>

#include "TestHelpers.h"

using namespace motorium;
using namespace motorium::drake_sim;
using namespace motorium::drake_sim::test;

namespace {

constexpr double kStandingHeight = 0.27;
const std::vector<std::string> kLegs = {"FL_leg", "FR_leg", "RL_leg", "RR_leg"};

class QuadrupedTest : public ::testing::Test {
 protected:
  void SetUp() override {
    hal_ = std::make_unique<hal::RobotHAL>(quadrupedUrdf());
    state_ = std::make_unique<model::RobotState>(hal_->getRobotDescription(), kLegs.size());
  }

  std::shared_ptr<model::RobotState> initialState(const vector3_t& position,
                                                  const quaternion_t& orientation = quaternion_t::Identity(),
                                                  const vector3_t& linear_velocity_local = vector3_t::Zero(),
                                                  const vector3_t& angular_velocity_local = vector3_t::Zero()) const {
    auto state = std::make_shared<model::RobotState>(hal_->getRobotDescription());
    state->setConfigurationToZero();
    state->setRootPositionInWorldFrame(position);
    state->setRootRotationLocalToWorldFrame(orientation);
    state->setRootLinearVelocityInLocalFrame(linear_velocity_local);
    state->setRootAngularVelocityInLocalFrame(angular_velocity_local);
    return state;
  }

  DrakeDriver& addDriver(DrakeSimConfig config) {
    config.manualStepping = true;
    return hal_->addDriver<DrakeDriver>(config);
  }

  std::unique_ptr<hal::RobotHAL> hal_;
  std::unique_ptr<model::RobotState> state_;
};

}  // namespace

TEST_F(QuadrupedTest, FreeFallFollowsBallisticMotion) {
  DrakeSimConfig config;
  config.ground.enabled = false;
  config.initStatePtr = initialState({0.0, 0.0, 1.0});
  DrakeDriver& sim = addDriver(config);
  hal_->startDrivers();

  const double duration = 0.5;
  ASSERT_TRUE(sim.advance(stepsFor(duration, config.dt)));
  hal_->updateState(*state_);

  // Discrete time stepping adds an O(g * dt * t) error (~1.2 mm here).
  EXPECT_NEAR(state_->getTime(), duration, 1e-9);
  EXPECT_NEAR(state_->getRootPositionInWorldFrame().z(), 1.0 - 0.5 * kGravity * duration * duration, 5e-3);
  EXPECT_NEAR(state_->getRootLinearVelocityInLocalFrame().z(), -kGravity * duration, 1e-6);
  EXPECT_NEAR(state_->getRootPositionInWorldFrame().x(), 0.0, 1e-9);
  hal_->stopDrivers();
}

// RobotState root velocities are in the local (root) frame; Drake works in the world frame. A wrong
// conversion in either direction shows up here.
TEST_F(QuadrupedTest, RootLinearVelocityIsInTheLocalFrame) {
  DrakeSimConfig config;
  config.ground.enabled = false;
  // Yawed by 90 deg: local +x points along world +y.
  const quaternion_t yaw_90(Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ()));
  config.initStatePtr = initialState({0.0, 0.0, 1.0}, yaw_90, {1.0, 0.0, 0.0});
  DrakeDriver& sim = addDriver(config);
  hal_->startDrivers();

  const double duration = 0.1;
  ASSERT_TRUE(sim.advance(stepsFor(duration, config.dt)));
  hal_->updateState(*state_);

  EXPECT_NEAR(state_->getRootPositionInWorldFrame().x(), 0.0, 1e-6);
  EXPECT_NEAR(state_->getRootPositionInWorldFrame().y(), duration, 1e-6);
  EXPECT_NEAR(state_->getRootLinearVelocityInLocalFrame().x(), 1.0, 1e-6);
  EXPECT_NEAR(state_->getRootLinearVelocityInLocalFrame().y(), 0.0, 1e-6);
  EXPECT_TRUE(state_->getRootRotationLocalToWorldFrame().isApprox(yaw_90, 1e-9));
  hal_->stopDrivers();
}

TEST_F(QuadrupedTest, RootAngularVelocityIsInTheLocalFrame) {
  DrakeSimConfig config;
  config.ground.enabled = false;
  config.defaultJointDamping = 0.0;
  // Rolled by 90 deg: local +z points along world -y. Spin about the local z axis.
  const quaternion_t roll_90(Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitX()));
  const double spin_rate = 2.0;
  config.initStatePtr = initialState({0.0, 0.0, 1.0}, roll_90, vector3_t::Zero(), {0.0, 0.0, spin_rate});
  DrakeDriver& sim = addDriver(config);
  hal_->startDrivers();

  const double duration = 0.05;
  ASSERT_TRUE(sim.advance(stepsFor(duration, config.dt)));
  hal_->updateState(*state_);

  const vector3_t angular_velocity_local = state_->getRootAngularVelocityInLocalFrame();
  EXPECT_NEAR(angular_velocity_local.x(), 0.0, 1e-2);
  EXPECT_NEAR(angular_velocity_local.y(), 0.0, 1e-2);
  EXPECT_NEAR(angular_velocity_local.z(), spin_rate, 1e-2);

  // The orientation advanced by spin_rate * duration about the local z axis.
  const quaternion_t expected = roll_90 * quaternion_t(Eigen::AngleAxisd(spin_rate * duration, Eigen::Vector3d::UnitZ()));
  EXPECT_LT(state_->getRootRotationLocalToWorldFrame().angularDistance(expected), 1e-2);
  hal_->stopDrivers();
}

TEST_F(QuadrupedTest, StandsUnderPdHoldWithFeetInContact) {
  DrakeSimConfig config;
  config.contactBodyNames = kLegs;
  config.initStatePtr = initialState({0.0, 0.0, kStandingHeight + 0.01});
  DrakeDriver& sim = addDriver(config);

  const std::vector<std::string> hips = {"FL_hip", "FR_hip", "RL_hip", "RR_hip"};
  control::ImplicitJointPDController controller(hal_->getRobotDescription(), {hips, {100.0, 100.0, 100.0, 100.0}, {2.0, 2.0, 2.0, 2.0}});
  model::RobotState desired(hal_->getRobotDescription());
  desired.setConfigurationToZero();
  model::RobotJointFeedbackAction action(hal_->getRobotDescription());

  hal_->startDrivers();
  for (size_t step = 0; step < stepsFor(3.0, config.dt); ++step) {
    hal_->updateState(*state_);
    controller.computeJointControlAction(state_->getTime(), *state_, desired, action);
    hal_->setAction(action);
    ASSERT_TRUE(sim.advance()) << "faulted at t = " << state_->getTime();
  }
  hal_->updateState(*state_);

  // Point contact with 5e4 N/m stiffness sinks the feet by a few millimetres.
  EXPECT_NEAR(state_->getRootPositionInWorldFrame().z(), kStandingHeight, 0.01);
  EXPECT_NEAR(rollOf(state_->getRootRotationLocalToWorldFrame()), 0.0, 0.02);
  EXPECT_NEAR(pitchOf(state_->getRootRotationLocalToWorldFrame()), 0.0, 0.02);
  EXPECT_LT(state_->getRootLinearVelocityInLocalFrame().norm(), 0.01);
  for (size_t foot = 0; foot < kLegs.size(); ++foot) {
    EXPECT_TRUE(state_->getContactFlag(foot)) << kLegs[foot];
  }
  hal_->stopDrivers();
}

TEST_F(QuadrupedTest, ContactFlagsAreFalseInTheAir) {
  DrakeSimConfig config;
  config.contactBodyNames = kLegs;
  config.initStatePtr = initialState({0.0, 0.0, 1.0});
  DrakeDriver& sim = addDriver(config);
  hal_->startDrivers();
  ASSERT_TRUE(sim.advance(10));
  hal_->updateState(*state_);
  for (size_t foot = 0; foot < kLegs.size(); ++foot) {
    EXPECT_FALSE(state_->getContactFlag(foot)) << kLegs[foot];
  }
  hal_->stopDrivers();
}

TEST_F(QuadrupedTest, ContactFlagsAreLeftUntouchedWithoutContactBodies) {
  DrakeSimConfig config;
  config.initStatePtr = initialState({0.0, 0.0, 1.0});
  DrakeDriver& sim = addDriver(config);
  hal_->startDrivers();
  state_->setConfigurationToZero();  // all contact flags true
  ASSERT_TRUE(sim.advance(10));
  hal_->updateState(*state_);
  for (size_t foot = 0; foot < kLegs.size(); ++foot) {
    EXPECT_TRUE(state_->getContactFlag(foot));
  }
  hal_->stopDrivers();
}

TEST_F(QuadrupedTest, FallBelowRootHeightFaultsTheHalAndResetRecovers) {
  DrakeSimConfig config;
  config.ground.enabled = false;
  config.faultBelowRootHeight = 0.5;
  config.initStatePtr = initialState({0.0, 0.0, 1.0});
  DrakeDriver& sim = addDriver(config);
  hal_->startDrivers();

  // Falls 0.5 m in ~0.32 s.
  EXPECT_FALSE(sim.advance(stepsFor(1.0, config.dt)));
  EXPECT_EQ(sim.getState(), hal::DriverState::FAULT);
  sim.copyStateSnapshot(*state_);
  EXPECT_LT(state_->getRootPositionInWorldFrame().z(), 0.5);
  EXPECT_GT(state_->getRootPositionInWorldFrame().z(), 0.45);  // stopped right at the threshold

  // The HAL notices on its next call and latches a safety stop.
  hal_->updateState(*state_);
  EXPECT_EQ(hal_->getState(), hal::DriverState::FAULT);

  hal_->resetDrivers();
  EXPECT_EQ(sim.getState(), hal::DriverState::READY);
  sim.copyStateSnapshot(*state_);
  EXPECT_NEAR(state_->getRootPositionInWorldFrame().z(), 1.0, 1e-12);

  hal_->startDrivers();
  EXPECT_EQ(hal_->getState(), hal::DriverState::RUNNING);
  EXPECT_TRUE(sim.advance(10));
  hal_->stopDrivers();
}
