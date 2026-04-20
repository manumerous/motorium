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

using namespace motorium::hal;
using namespace motorium::model;

// Minimal concrete driver for testing.
class TestDriver : public DriverBase {
 public:
  using DriverBase::DriverBase;

  void start() override { transitionTo(DriverState::CONFIGURED); }
  void stop() override { transitionTo(DriverState::FAULT); }
  void updateRobotState(motorium::model::RobotState& state) override {
    update_count_++;
    (void)state;
  }
  void setJointFeedbackAction(const motorium::model::RobotJointFeedbackAction&) override { action_count_++; }
  void reset() override {}

  void triggerTransition(DriverState next) { transitionTo(next); }

  int update_count_{0};
  int action_count_{0};
};

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
    return std::make_shared<TestDriver>(desc, joints, name);
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
  EXPECT_DEATH(RobotHAL(xml_path_, {d}), "");
}

TEST_F(RobotHALTest, ConstructorThrowsOnDuplicateJointInDriver) {
  EXPECT_THROW(
      {
        auto d = makeDriver("driver", {"joint1", "joint1"});
        RobotHAL hal(xml_path_, {d});
      },
      std::invalid_argument);
}

TEST_F(RobotHALTest, ConstructorThrowsOnDuplicateManagedJointInDriver) {
  std::vector<JointDescription> joint_descs;
  for (const auto& name : {"joint1", "joint2"}) {
    JointDescription jd;
    jd.name = name;
    joint_descs.push_back(jd);
  }
  RobotDescription desc(joint_descs);
  auto d = std::make_shared<TestDriver>(desc, std::vector<std::string>{"joint1", "joint1"}, "driver");
  EXPECT_THROW(RobotHAL(xml_path_, {d}), std::runtime_error);
}

TEST_F(RobotHALTest, ConstructorThrowsOnNullDriver) {
  auto d = makeDriver("driver", {"joint1", "joint2"});
  EXPECT_THROW(RobotHAL(xml_path_, {d, nullptr}), std::runtime_error);
}

TEST_F(RobotHALTest, ConstructorThrowsOnOverlappingDrivers) {
  auto d1 = makeDriver("driver1", {"joint1", "joint2"});
  auto d2 = makeDriver("driver2", {"joint2"});  // joint2 covered twice
  EXPECT_THROW(RobotHAL(xml_path_, {d1, d2}), std::runtime_error);
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

TEST_F(RobotHALTest, InitialStateIsUnconfigured) {
  auto d = makeDriver("driver", {"joint1", "joint2"});
  RobotHAL hal(xml_path_, {d});
  EXPECT_EQ(hal.getState(), HalState::UNCONFIGURED);
}

TEST_F(RobotHALTest, StateIsConfiguredWhenAllDriversConfigured) {
  auto d1 = makeDriver("d1", {"joint1"});
  auto d2 = makeDriver("d2", {"joint2"});
  RobotHAL hal(xml_path_, {d1, d2});

  d1->triggerTransition(DriverState::CONFIGURED);
  d2->triggerTransition(DriverState::CONFIGURED);
  EXPECT_EQ(hal.getState(), HalState::CONFIGURED);
}

TEST_F(RobotHALTest, StateIsConfiguredWhenDriversMixedConfiguredAndReady) {
  auto d1 = makeDriver("d1", {"joint1"});
  auto d2 = makeDriver("d2", {"joint2"});
  RobotHAL hal(xml_path_, {d1, d2});

  d1->triggerTransition(DriverState::CONFIGURED);
  d2->triggerTransition(DriverState::CONFIGURED);
  d2->triggerTransition(DriverState::READY);
  EXPECT_EQ(hal.getState(), HalState::CONFIGURED);
}

TEST_F(RobotHALTest, StateIsActiveWhenAllDriversRunning) {
  auto d1 = makeDriver("d1", {"joint1"});
  auto d2 = makeDriver("d2", {"joint2"});
  RobotHAL hal(xml_path_, {d1, d2});

  for (auto* d : {d1.get(), d2.get()}) {
    d->triggerTransition(DriverState::CONFIGURED);
    d->triggerTransition(DriverState::READY);
    d->triggerTransition(DriverState::RUNNING);
  }
  EXPECT_EQ(hal.getState(), HalState::ACTIVE);
}

TEST_F(RobotHALTest, StateIsFaultWhenAnyDriverInFault) {
  auto d1 = makeDriver("d1", {"joint1"});
  auto d2 = makeDriver("d2", {"joint2"});
  RobotHAL hal(xml_path_, {d1, d2});

  d1->triggerTransition(DriverState::CONFIGURED);
  d1->triggerTransition(DriverState::READY);
  d1->triggerTransition(DriverState::RUNNING);
  // d2 remains UNINITIALIZED, but d1 goes FAULT
  d1->triggerTransition(DriverState::FAULT);
  EXPECT_EQ(hal.getState(), HalState::FAULT);
}

// ─── update ─────────────────────────────────────────────────────────────────

TEST_F(RobotHALTest, updateCallsAllDrivers) {
  auto d1 = makeDriver("d1", {"joint1"});
  auto d2 = makeDriver("d2", {"joint2"});
  RobotHAL hal(xml_path_, {d1, d2});

  RobotState state(hal.getRobotDescription());
  RobotJointFeedbackAction action(hal.getRobotDescription());

  hal.update(action, state);

  EXPECT_EQ(d1->action_count_, 1);
  EXPECT_EQ(d1->update_count_, 1);
  EXPECT_EQ(d2->action_count_, 1);
  EXPECT_EQ(d2->update_count_, 1);
}
