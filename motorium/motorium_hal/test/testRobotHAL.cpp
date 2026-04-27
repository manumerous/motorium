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
  EXPECT_THROW(hal.addDriver<TestDriver>("driver", std::vector<std::string>{"joint1", "joint1"}),
               std::invalid_argument);
}

TEST_F(RobotHALTest, StartDriversKillsOnOverlappingDrivers) {
  RobotHAL hal(xml_path_);
  hal.addDriver<TestDriver>("driver1", std::vector<std::string>{"joint1", "joint2"});
  hal.addDriver<TestDriver>("driver2", std::vector<std::string>{"joint2"});  // joint2 twice
  EXPECT_DEATH(hal.startDrivers(), "managed by multiple drivers");
}

TEST_F(RobotHALTest, AddDriverThrowsOnJointNotInDescription) {
  RobotHAL hal(xml_path_);
  EXPECT_THROW(hal.addDriver<TestDriver>("driver", std::vector<std::string>{"joint1", "ghost_joint"}),
               std::out_of_range);
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

TEST_F(RobotHALTest, StateIsStoppingAfterStopDrivers) {
  RobotHAL hal(xml_path_);
  hal.addDriver<TestDriver>("driver");
  hal.startDrivers();
  hal.stopDrivers();
  EXPECT_EQ(hal.getState(), DriverState::STOPPING);
}

TEST_F(RobotHALTest, StateIsFaultWhenAnyDriverInFault) {
  RobotHAL hal(xml_path_);
  TestDriver& d1 = hal.addDriver<TestDriver>("d1", std::vector<std::string>{"joint1"});
  TestDriver& d2 = hal.addDriver<TestDriver>("d2", std::vector<std::string>{"joint2"});
  hal.startDrivers();

  d1.triggerTransition(DriverState::FAULT);

  EXPECT_EQ(hal.getState(), DriverState::FAULT);
  (void)d2;
}

// ─── update ─────────────────────────────────────────────────────────────────

TEST_F(RobotHALTest, UpdateCallsAllRunningDrivers) {
  RobotHAL hal(xml_path_);
  TestDriver& d1 = hal.addDriver<TestDriver>("d1", std::vector<std::string>{"joint1"});
  TestDriver& d2 = hal.addDriver<TestDriver>("d2", std::vector<std::string>{"joint2"});

  RobotState state(hal.getRobotDescription());
  RobotJointFeedbackAction action(hal.getRobotDescription());

  hal.startDrivers();
  hal.update(action, state);

  EXPECT_EQ(d1.update_count_, 1);
  EXPECT_EQ(d2.update_count_, 1);
}
