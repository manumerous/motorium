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

#include <chrono>
#include <memory>
#include <thread>

#include "TestHelpers.h"

using namespace motorium;
using namespace motorium::drake_sim;
using namespace motorium::drake_sim::test;

namespace {

DrakeSimConfig manualConfig() {
  DrakeSimConfig config;
  config.manualStepping = true;
  return config;
}

}  // namespace

TEST(DrakeDriverLifecycleTest, ConstructsReadyAndLoadsTheHalModelByDefault) {
  hal::RobotHAL hal(pendulumUrdf());
  DrakeDriver& sim = hal.addDriver<DrakeDriver>(manualConfig());  // modelPath empty -> HAL's URDF
  EXPECT_EQ(sim.getState(), hal::DriverState::READY);
  EXPECT_EQ(hal.getState(), hal::DriverState::READY);
  EXPECT_TRUE(sim.getMeshcatUrl().empty());
}

TEST(DrakeDriverLifecycleTest, StartStopAndRestart) {
  hal::RobotHAL hal(pendulumUrdf());
  DrakeDriver& sim = hal.addDriver<DrakeDriver>(manualConfig());

  hal.startDrivers();
  EXPECT_EQ(sim.getState(), hal::DriverState::RUNNING);
  hal.stopDrivers();
  EXPECT_EQ(sim.getState(), hal::DriverState::READY);
  hal.startDrivers();
  EXPECT_EQ(sim.getState(), hal::DriverState::RUNNING);
  hal.stopDrivers();
}

TEST(DrakeDriverLifecycleTest, PhysicsDoesNotAdvanceBeforeStart) {
  hal::RobotHAL hal(pendulumUrdf());
  DrakeDriver& sim = hal.addDriver<DrakeDriver>(manualConfig());

  EXPECT_FALSE(sim.advance(10));  // READY, not RUNNING
  model::RobotState state(hal.getRobotDescription());
  sim.copyStateSnapshot(state);
  EXPECT_DOUBLE_EQ(state.getTime(), 0.0);
  EXPECT_EQ(sim.getMetrics().stepCount, 0u);
}

TEST(DrakeDriverLifecycleTest, ManualSteppingAdvancesTimeExactly) {
  hal::RobotHAL hal(pendulumUrdf());
  DrakeDriver& sim = hal.addDriver<DrakeDriver>(manualConfig());
  hal.startDrivers();

  ASSERT_TRUE(sim.advance(200));
  model::RobotState state(hal.getRobotDescription());
  hal.updateState(state);
  EXPECT_NEAR(state.getTime(), 200 * sim.getConfig().dt, 1e-9);
  EXPECT_EQ(sim.getMetrics().stepCount, 200u);
  hal.stopDrivers();
}

TEST(DrakeDriverLifecycleTest, RealTimeThreadStepsAtRoughlyRealTime) {
  DrakeSimConfig config;  // physics thread
  hal::RobotHAL hal(pendulumUrdf());
  DrakeDriver& sim = hal.addDriver<DrakeDriver>(config);

  hal.startDrivers();
  const auto start = std::chrono::steady_clock::now();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  model::RobotState state(hal.getRobotDescription());
  hal.updateState(state);
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  hal.stopDrivers();

  // A 1-DoF pendulum steps far faster than real time, so pacing decides: sim time ~ wall time.
  EXPECT_GT(state.getTime(), 0.5 * wall);
  EXPECT_LT(state.getTime(), 1.2 * wall);
  EXPECT_GT(sim.getMetrics().stepCount, 0u);
}

TEST(DrakeDriverLifecycleTest, ResetRestoresTheInitialState) {
  auto init_state = std::make_shared<model::RobotState>(model::RobotDescription(pendulumUrdf()).getNumJoints());
  init_state->setConfigurationToZero();
  init_state->setJointPosition(0, 0.3);

  DrakeSimConfig config = manualConfig();
  config.defaultJointDamping = 0.0;  // with damping 10 the pendulum is heavily overdamped and barely moves
  config.initStatePtr = init_state;
  hal::RobotHAL hal(pendulumUrdf());
  DrakeDriver& sim = hal.addDriver<DrakeDriver>(config);
  hal.startDrivers();

  model::RobotState state(hal.getRobotDescription());
  hal.updateState(state);
  EXPECT_NEAR(state.getJointPosition(0), 0.3, 1e-12);

  ASSERT_TRUE(sim.advance(stepsFor(0.3, config.dt)));
  hal.updateState(state);
  EXPECT_LT(state.getJointPosition(0), 0.25);  // swung down

  hal.resetDrivers();  // allowed while RUNNING
  EXPECT_EQ(sim.getState(), hal::DriverState::RUNNING);
  hal.updateState(state);
  EXPECT_NEAR(state.getJointPosition(0), 0.3, 1e-12);
  EXPECT_NEAR(state.getJointVelocity(0), 0.0, 1e-12);
  hal.stopDrivers();
}

TEST(DrakeDriverLifecycleDeathTest, JointMissingFromTheModelAborts) {
  const std::vector<model::JointDescription> joints = {
      {"hinge", {-10.0, 10.0}, {-100.0, 100.0}, {-5.0, 5.0}},
      {"not_in_model", {-10.0, 10.0}, {-100.0, 100.0}, {-5.0, 5.0}},
  };
  DrakeSimConfig config = manualConfig();
  config.modelPath = pendulumUrdf();
  EXPECT_DEATH(
      {
        hal::RobotHAL hal(std::make_unique<model::RobotDescription>(joints));
        hal.addDriver<DrakeDriver>(config);
      },
      "not_in_model.*is not in the Drake model");
}

TEST(DrakeDriverLifecycleDeathTest, NoModelPathAborts) {
  const std::vector<model::JointDescription> joints = {{"hinge", {-10.0, 10.0}, {-100.0, 100.0}, {-5.0, 5.0}}};
  EXPECT_DEATH(
      {
        hal::RobotHAL hal(std::make_unique<model::RobotDescription>(joints));
        hal.addDriver<DrakeDriver>(manualConfig());
      },
      "No model to load");
}

TEST(DrakeDriverLifecycleDeathTest, AdvanceRequiresManualStepping) {
  EXPECT_DEATH(
      {
        hal::RobotHAL hal(pendulumUrdf());
        DrakeDriver& sim = hal.addDriver<DrakeDriver>(DrakeSimConfig{});
        sim.advance(1);
      },
      "requires DrakeSimConfig::manualStepping");
}
