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

#include <memory>
#include <vector>

#include "TestDriver.h"

using namespace motorium::hal;
using namespace motorium::model;

class DriverBaseTest : public ::testing::Test {
 protected:
  void SetUp() override {
    JointDescription j1;
    j1.name = "joint1";
    JointDescription j2;
    j2.name = "joint2";
    hal_ = std::make_unique<RobotHAL>(std::make_unique<RobotDescription>(std::vector<JointDescription>{j1, j2}));
  }

  std::unique_ptr<RobotHAL> hal_;
};

TEST_F(DriverBaseTest, InitialStateIsREADY) {
  TestDriver& driver = hal_->addDriver<TestDriver>("test_driver");
  EXPECT_EQ(driver.getState(), DriverState::READY);
}

TEST_F(DriverBaseTest, GetNameReturnsConstructorArg) {
  TestDriver& driver = hal_->addDriver<TestDriver>("my_driver");
  EXPECT_EQ(driver.getName(), "my_driver");
}

TEST_F(DriverBaseTest, AllJointsManaged) {
  TestDriver& driver = hal_->addDriver<TestDriver>("test_driver");
  const auto& joints = driver.getManagedJointNames();
  ASSERT_EQ(joints.size(), 2u);
  EXPECT_EQ(joints[0], "joint1");
  EXPECT_EQ(joints[1], "joint2");
}

TEST_F(DriverBaseTest, ExplicitSubsetJointsManaged) {
  TestDriver& driver = hal_->addDriver<TestDriver>("test_driver", std::vector<std::string>{"joint2"});
  const auto& joints = driver.getManagedJointNames();
  ASSERT_EQ(joints.size(), 1u);
  EXPECT_EQ(joints[0], "joint2");
}

TEST_F(DriverBaseTest, LegalTransitionSequence) {
  TestDriver& driver = hal_->addDriver<TestDriver>("test_driver");
  driver.requestTransitionTo(DriverState::RUNNING);
  driver.requestTransitionTo(DriverState::STOPPING);
  driver.requestTransitionTo(DriverState::READY);
  EXPECT_EQ(driver.getState(), DriverState::READY);
}

TEST_F(DriverBaseTest, FaultTransitionFromRunning) {
  TestDriver& driver = hal_->addDriver<TestDriver>("test_driver", std::vector<std::string>{"joint1", "joint2"});
  driver.requestTransitionTo(DriverState::RUNNING);
  driver.requestTransitionTo(DriverState::FAULT);
  EXPECT_EQ(driver.getState(), DriverState::FAULT);
}

TEST_F(DriverBaseTest, FaultTransitionFromStopping) {
  TestDriver& driver = hal_->addDriver<TestDriver>("test_driver");
  driver.requestTransitionTo(DriverState::RUNNING);
  driver.requestTransitionTo(DriverState::STOPPING);
  driver.requestTransitionTo(DriverState::FAULT);
  EXPECT_EQ(driver.getState(), DriverState::FAULT);
}

TEST_F(DriverBaseTest, AddDriverThrowsOnInvalidJointName) {
  EXPECT_THROW(hal_->addDriver<TestDriver>("test_driver", std::vector<std::string>{"nonexistent_joint"}), std::out_of_range);
}

TEST_F(DriverBaseTest, IllegalTransitionKills) {
  TestDriver& driver = hal_->addDriver<TestDriver>("test_driver");
  // READY -> STOPPING is not a legal transition
  EXPECT_DEATH(driver.requestTransitionTo(DriverState::STOPPING), "Illegal state transition");
}
