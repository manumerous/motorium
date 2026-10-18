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
#include <motorium_hal/RobotHAL.h>
#include <motorium_model/RobotDescription.h>
#include <motorium_model/RobotJointFeedbackAction.h>
#include <motorium_model/RobotState.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <vector>

#include "TestDriver.h"

using namespace motorium::hal;
using namespace motorium::model;

// Writes a minimal MuJoCo XML with the given joint names to a temp file.
static std::string writeTempXml(const std::filesystem::path& dir,
                                const std::string& filename,
                                const std::vector<std::string>& joint_names) {
  std::filesystem::path path = dir / filename;
  std::ofstream f(path);
  f << "<mujoco model=\"test_robot\">\n<worldbody>\n<body name=\"root\">\n";
  for (const auto& name : joint_names) {
    f << "  <joint name=\"" << name << "\" type=\"hinge\" range=\"-1.57 1.57\" actuatorfrcrange=\"-100 100\"/>\n";
  }
  f << "</body>\n</worldbody>\n<actuator>\n";
  for (const auto& name : joint_names) {
    f << "  <motor name=\"" << name << "\" joint=\"" << name << "\"/>\n";
  }
  f << "</actuator>\n</mujoco>\n";
  return path.string();
}

class RobotHALTest : public ::testing::Test {
 protected:
  void SetUp() override {
    temp_dir_ = std::filesystem::temp_directory_path() / "robot_hal_test";
    std::filesystem::create_directories(temp_dir_);
    xml_path_ = writeTempXml(temp_dir_, "robot.xml", {"joint1", "joint2"});
  }

  void TearDown() override { std::filesystem::remove_all(temp_dir_); }

  std::filesystem::path temp_dir_;
  std::string xml_path_;
};

// ─── addDriver / startDrivers validation ──────────────────────────────────────

TEST_F(RobotHALTest, StartDriversSucceedsWhenAllJointsCovered) {
  RobotHAL hal(xml_path_);
  hal.addDriver<TestDriver>("driver1", std::vector<std::string>{"joint1"});
  hal.addDriver<TestDriver>("driver2", std::vector<std::string>{"joint2"});
  EXPECT_NO_FATAL_FAILURE(hal.startDrivers());
}

TEST_F(RobotHALTest, StartDriversSucceedsWithSingleDriverCoveringAllJoints) {
  RobotHAL hal(xml_path_);
  hal.addDriver<TestDriver>("driver");  // all-joints constructor
  EXPECT_NO_FATAL_FAILURE(hal.startDrivers());
}

TEST_F(RobotHALTest, StartDriversKillsOnUncoveredJoint) {
  RobotHAL hal(xml_path_);
  hal.addDriver<TestDriver>("driver", std::vector<std::string>{"joint1"});  // joint2 not covered
  EXPECT_DEATH(hal.startDrivers(), "not managed by any driver");
}

TEST_F(RobotHALTest, AddDriverThrowsOnDuplicateJointInList) {
  RobotHAL hal(xml_path_);
  EXPECT_THROW(hal.addDriver<TestDriver>("driver", std::vector<std::string>{"joint1", "joint1"}), std::invalid_argument);
}

TEST_F(RobotHALTest, StartDriversKillsOnOverlappingDrivers) {
  RobotHAL hal(xml_path_);
  hal.addDriver<TestDriver>("driver1", std::vector<std::string>{"joint1", "joint2"});
  hal.addDriver<TestDriver>("driver2", std::vector<std::string>{"joint2"});  // joint2 twice
  EXPECT_DEATH(hal.startDrivers(), "managed by multiple drivers");
}

TEST_F(RobotHALTest, AddDriverThrowsOnJointNotInDescription) {
  RobotHAL hal(xml_path_);
  EXPECT_THROW(hal.addDriver<TestDriver>("driver", std::vector<std::string>{"joint1", "ghost_joint"}), std::out_of_range);
}

// ─── getRobotDescription ──────────────────────────────────────────────────────

TEST_F(RobotHALTest, GetRobotDescriptionReturnsCorrectJoints) {
  RobotHAL hal(xml_path_);
  hal.addDriver<TestDriver>("driver");

  const auto& desc = hal.getRobotDescription();
  EXPECT_TRUE(desc.containsJoint("joint1"));
  EXPECT_TRUE(desc.containsJoint("joint2"));
  EXPECT_EQ(desc.getNumJoints(), 2u);
}

// ─── getState ────────────────────────────────────────────────────────────────

TEST_F(RobotHALTest, StateIsRunningAfterStartDrivers) {
  RobotHAL hal(xml_path_);
  hal.addDriver<TestDriver>("driver");
  EXPECT_EQ(hal.getState(), DriverState::READY);
  hal.startDrivers();
  EXPECT_EQ(hal.getState(), DriverState::RUNNING);
}

TEST_F(RobotHALTest, StateIsReadyAfterStopDrivers) {
  RobotHAL hal(xml_path_);
  hal.addDriver<TestDriver>("driver");
  hal.startDrivers();
  hal.stopDrivers();
  EXPECT_EQ(hal.getState(), DriverState::READY);
}

TEST_F(RobotHALTest, StateIsFaultWhenAnyDriverInFault) {
  RobotHAL hal(xml_path_);
  TestDriver& d1 = hal.addDriver<TestDriver>("d1", std::vector<std::string>{"joint1"});
  TestDriver& d2 = hal.addDriver<TestDriver>("d2", std::vector<std::string>{"joint2"});
  hal.startDrivers();

  d1.triggerFault();

  EXPECT_EQ(hal.getState(), DriverState::FAULT);
  (void)d2;
}

// ─── update ─────────────────────────────────────────────────────────────────

TEST_F(RobotHALTest, UpdateStateCallsAllRunningDrivers) {
  RobotHAL hal(xml_path_);
  TestDriver& d1 = hal.addDriver<TestDriver>("d1", std::vector<std::string>{"joint1"});
  TestDriver& d2 = hal.addDriver<TestDriver>("d2", std::vector<std::string>{"joint2"});

  RobotState state(hal.getRobotDescription());

  hal.startDrivers();
  hal.updateState(state);

  EXPECT_EQ(d1.update_state_count_, 1);
  EXPECT_EQ(d2.update_state_count_, 1);
  EXPECT_EQ(d1.set_action_count_, 0);
  EXPECT_EQ(d2.set_action_count_, 0);
}

TEST_F(RobotHALTest, SetActionCallsAllRunningDrivers) {
  RobotHAL hal(xml_path_);
  TestDriver& d1 = hal.addDriver<TestDriver>("d1", std::vector<std::string>{"joint1"});
  TestDriver& d2 = hal.addDriver<TestDriver>("d2", std::vector<std::string>{"joint2"});

  RobotJointFeedbackAction action(hal.getRobotDescription());

  hal.startDrivers();
  hal.setAction(action);

  EXPECT_EQ(d1.set_action_count_, 1);
  EXPECT_EQ(d2.set_action_count_, 1);
  EXPECT_EQ(d1.update_state_count_, 0);
  EXPECT_EQ(d2.update_state_count_, 0);
}

// ─── safety stop ────────────────────────────────────────────────────────────

class RobotHALSafetyTest : public RobotHALTest {
 protected:
  void SetUp() override {
    RobotHALTest::SetUp();
    hal_ = std::make_unique<RobotHAL>(xml_path_);
    d1_ = &hal_->addDriver<TestDriver>("d1", std::vector<std::string>{"joint1"});
    d2_ = &hal_->addDriver<TestDriver>("d2", std::vector<std::string>{"joint2"});
    state_ = std::make_unique<RobotState>(hal_->getRobotDescription());
    action_ = std::make_unique<RobotJointFeedbackAction>(hal_->getRobotDescription());
  }

  std::unique_ptr<RobotHAL> hal_;
  TestDriver* d1_{nullptr};
  TestDriver* d2_{nullptr};
  std::unique_ptr<RobotState> state_;
  std::unique_ptr<RobotJointFeedbackAction> action_;
};

TEST_F(RobotHALSafetyTest, UpdateStateWithFaultedDriverStopsAllDriversAndLatchesFault) {
  hal_->startDrivers();
  d2_->triggerFault();

  hal_->updateState(*state_);

  // d1 comes first and is still read; d2 refuses, every driver is stopped, and d1 is back at rest.
  EXPECT_EQ(d1_->update_state_count_, 1);
  EXPECT_EQ(d2_->update_state_count_, 0);
  EXPECT_EQ(d1_->stop_count_, 1);
  EXPECT_EQ(d2_->stop_count_, 1);
  EXPECT_EQ(d1_->getState(), DriverState::READY);
  EXPECT_EQ(d2_->getState(), DriverState::FAULT);
  EXPECT_EQ(hal_->getState(), DriverState::FAULT);
}

TEST_F(RobotHALSafetyTest, SetActionWithFaultedDriverSendsNothing) {
  hal_->startDrivers();
  d1_->triggerFault();

  hal_->setAction(*action_);

  EXPECT_EQ(d1_->set_action_count_, 0);
  EXPECT_EQ(d2_->set_action_count_, 0);
  EXPECT_EQ(d2_->getState(), DriverState::READY);
  EXPECT_EQ(hal_->getState(), DriverState::FAULT);
}

TEST_F(RobotHALSafetyTest, FaultDuringReadStopsBeforeTheActionIsSent) {
  hal_->startDrivers();
  d1_->fault_on_update_state_ = true;

  hal_->updateState(*state_);
  hal_->setAction(*action_);

  EXPECT_EQ(hal_->getState(), DriverState::FAULT);
  EXPECT_EQ(d2_->getState(), DriverState::READY);
  // Stopped right after d1's read failed, so d2 was never read and nothing was sent.
  EXPECT_EQ(d2_->update_state_count_, 0);
  EXPECT_EQ(d1_->set_action_count_, 0);
  EXPECT_EQ(d2_->set_action_count_, 0);
}

TEST_F(RobotHALSafetyTest, FaultDuringSendStopsBeforeRemainingDriversAreCommanded) {
  hal_->startDrivers();
  d1_->fault_on_set_action_ = true;

  hal_->setAction(*action_);

  // Caught on the same tick: d2 never receives the action and is brought to rest.
  EXPECT_EQ(hal_->getState(), DriverState::FAULT);
  EXPECT_EQ(d1_->set_action_count_, 1);
  EXPECT_EQ(d2_->set_action_count_, 0);
  EXPECT_EQ(d2_->stop_count_, 1);
  EXPECT_EQ(d2_->getState(), DriverState::READY);
}

TEST_F(RobotHALSafetyTest, LatchedHalIgnoresFurtherCalls) {
  hal_->startDrivers();
  d2_->triggerFault();
  hal_->updateState(*state_);

  hal_->updateState(*state_);
  hal_->setAction(*action_);

  // Stopped exactly once; no further reads or writes after the first (pre-stop) read of d1.
  EXPECT_EQ(d1_->stop_count_, 1);
  EXPECT_EQ(d1_->update_state_count_, 1);
  EXPECT_EQ(d1_->set_action_count_, 0);
}

TEST_F(RobotHALSafetyTest, StartDriversKillsWhileLatched) {
  hal_->startDrivers();
  d2_->triggerFault();
  hal_->updateState(*state_);

  EXPECT_DEATH(hal_->startDrivers(), "call resetDrivers\\(\\) first");
}

TEST_F(RobotHALSafetyTest, ResetClearsTheLatchAndAllowsRestart) {
  hal_->startDrivers();
  d2_->triggerFault();
  hal_->updateState(*state_);

  hal_->resetDrivers();
  EXPECT_EQ(hal_->getState(), DriverState::READY);

  const int d1_reads_before_restart = d1_->update_state_count_;
  hal_->startDrivers();
  hal_->updateState(*state_);
  hal_->setAction(*action_);
  EXPECT_EQ(hal_->getState(), DriverState::RUNNING);
  EXPECT_EQ(d1_->update_state_count_, d1_reads_before_restart + 1);
  EXPECT_EQ(d2_->set_action_count_, 1);
}

TEST_F(RobotHALSafetyTest, DriverFaultingOnStartStopsAllDrivers) {
  d1_->fault_on_start_ = true;

  hal_->startDrivers();

  EXPECT_EQ(hal_->getState(), DriverState::FAULT);
  EXPECT_EQ(d1_->getState(), DriverState::FAULT);
  // d2 started fine but must not keep running on its own.
  EXPECT_EQ(d2_->start_count_, 1);
  EXPECT_EQ(d2_->stop_count_, 1);
  EXPECT_EQ(d2_->getState(), DriverState::READY);
}

TEST_F(RobotHALSafetyTest, UpdateStateBeforeStartTriggersSafetyStop) {
  hal_->updateState(*state_);

  EXPECT_EQ(hal_->getState(), DriverState::FAULT);
  EXPECT_EQ(d1_->update_state_count_, 0);
}

TEST_F(RobotHALSafetyTest, HealthyLoopNeverTriggersSafetyStop) {
  hal_->startDrivers();
  for (int i = 0; i < 10; ++i) {
    hal_->updateState(*state_);
    hal_->setAction(*action_);
  }
  EXPECT_EQ(hal_->getState(), DriverState::RUNNING);
  EXPECT_EQ(d1_->stop_count_, 0);
  EXPECT_EQ(d1_->update_state_count_, 10);
  EXPECT_EQ(d2_->set_action_count_, 10);
}
