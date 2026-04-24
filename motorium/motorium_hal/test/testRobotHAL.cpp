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

  std::shared_ptr<TestDriver> makeDriver(const std::string& name, std::vector<std::string> joints) {
    std::vector<JointDescription> joint_descs;
    for (const auto& j : joints) {
      JointDescription jd;
      jd.name = j;
      joint_descs.push_back(jd);
    }
    RobotDescription desc(joint_descs);
    return std::make_shared<TestDriver>(desc, name, joints);
  }

  std::filesystem::path temp_dir_;
  std::string xml_path_;
};

// ─── Constructor ──────────────────────────────────────────────────────────────

TEST_F(RobotHALTest, ConstructorSucceedsWhenAllJointsCovered) {
  auto d1 = makeDriver("driver1", {"joint1"});
  auto d2 = makeDriver("driver2", {"joint2"});
  EXPECT_NO_FATAL_FAILURE(RobotHAL hal(xml_path_, {d1, d2}));
}

TEST_F(RobotHALTest, ConstructorSucceedsWithSingleDriverCoveringAllJoints) {
  auto d = makeDriver("driver", {"joint1", "joint2"});
  EXPECT_NO_FATAL_FAILURE(RobotHAL hal(xml_path_, {d}));
}

TEST_F(RobotHALTest, ConstructorKillsOnUncoveredJoint) {
  auto d = makeDriver("driver", {"joint1"});  // joint2 not covered
  EXPECT_DEATH(RobotHAL(xml_path_, {d}), "not managed by any driver");
}

TEST_F(RobotHALTest, ConstructorThrowsOnDuplicateJointInDriver) {
  EXPECT_THROW(
      {
        auto d = makeDriver("driver", {"joint1", "joint1"});
        RobotHAL hal(xml_path_, {d});
      },
      std::invalid_argument);
}

TEST_F(RobotHALTest, ConstructorKillsOnDuplicateManagedJointInDriver) {
  std::vector<JointDescription> joint_descs;
  for (const auto& name : {"joint1", "joint2"}) {
    JointDescription jd;
    jd.name = name;
    joint_descs.push_back(jd);
  }
  RobotDescription desc(joint_descs);
  auto d = std::make_shared<TestDriver>(desc, "driver", std::vector<std::string>{"joint1", "joint1"});
  EXPECT_DEATH(RobotHAL(xml_path_, {d}), "managed by multiple drivers");
}

TEST_F(RobotHALTest, ConstructorKillsOnNullDriver) {
  auto d = makeDriver("driver", {"joint1", "joint2"});
  EXPECT_DEATH(RobotHAL(xml_path_, {d, nullptr}), "null driver");
}

TEST_F(RobotHALTest, ConstructorKillsOnOverlappingDrivers) {
  auto d1 = makeDriver("driver1", {"joint1", "joint2"});
  auto d2 = makeDriver("driver2", {"joint2"});  // joint2 covered twice
  EXPECT_DEATH(RobotHAL(xml_path_, {d1, d2}), "managed by multiple drivers");
}

TEST_F(RobotHALTest, ConstructorKillsOnDriverManagingJointNotInDescription) {
  // Driver claims to manage "ghost_joint" which is not in the HAL's xml model.
  auto d = makeDriver("driver", {"joint1", "joint2", "ghost_joint"});
  EXPECT_DEATH(RobotHAL(xml_path_, {d}), "not present in the robot description");
}

// ─── getRobotDescription ──────────────────────────────────────────────────────

TEST_F(RobotHALTest, GetRobotDescriptionReturnsCorrectJoints) {
  auto d = makeDriver("driver", {"joint1", "joint2"});
  RobotHAL hal(xml_path_, {d});

  const auto& desc = hal.getRobotDescription();
  EXPECT_TRUE(desc.containsJoint("joint1"));
  EXPECT_TRUE(desc.containsJoint("joint2"));
  EXPECT_EQ(desc.getNumJoints(), 2u);
}

// ─── getState ────────────────────────────────────────────────────────────────

TEST_F(RobotHALTest, StateIsRunningAfterStartDrivers) {
  auto d = makeDriver("driver", {"joint1", "joint2"});
  RobotHAL hal(xml_path_, {d});
  EXPECT_EQ(hal.getState(), DriverState::READY);
  hal.startDrivers();
  EXPECT_EQ(hal.getState(), DriverState::RUNNING);
}

TEST_F(RobotHALTest, StateIsStoppingAfterStopDrivers) {
  auto d = makeDriver("driver", {"joint1", "joint2"});
  RobotHAL hal(xml_path_, {d});
  hal.startDrivers();
  hal.stopDrivers();
  EXPECT_EQ(hal.getState(), DriverState::STOPPING);
}

TEST_F(RobotHALTest, StateIsFaultWhenDriverFaultDetectedDuringUpdate) {
  auto d1 = makeDriver("d1", {"joint1"});
  auto d2 = makeDriver("d2", {"joint2"});
  RobotHAL hal(xml_path_, {d1, d2});
  hal.startDrivers();

  d1->triggerTransition(DriverState::FAULT);

  RobotState state(hal.getRobotDescription());
  RobotJointFeedbackAction action(hal.getRobotDescription());
  hal.update(action, state);

  EXPECT_EQ(hal.getState(), DriverState::FAULT);
}

// ─── update ─────────────────────────────────────────────────────────────────

TEST_F(RobotHALTest, updateCallsAllDrivers) {
  auto d1 = makeDriver("d1", {"joint1"});
  auto d2 = makeDriver("d2", {"joint2"});
  RobotHAL hal(xml_path_, {d1, d2});

  RobotState state(hal.getRobotDescription());
  RobotJointFeedbackAction action(hal.getRobotDescription());

  hal.startDrivers();
  hal.update(action, state);

  EXPECT_EQ(d1->update_count_, 1);
  EXPECT_EQ(d2->update_count_, 1);
}
