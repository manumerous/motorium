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

#include <motorium_model/RobotState.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>

namespace motorium::model::testing {

class RobotDescriptionTest : public ::testing::Test {
 protected:
  // Create a temporary URDF file for testing
  void SetUp() override {
    tempDir_ = std::filesystem::temp_directory_path() / "robot_description_test";
    std::filesystem::create_directories(tempDir_);
    urdf_path_ = tempDir_ / "test_robot.urdf";

    // Create a simple test URDF with a few joints
    std::ofstream urdfFile(urdf_path_);
    urdfFile << R"(<?xml version="1.0"?>
        <robot name="test_robot">
            <link name="base_link"/>
            <link name="shoulder_link"/>
            <link name="elbow_link"/>
            <link name="wrist_link"/>
            <link name="hand_link"/>
            
            <joint name="shoulder_joint" type="revolute">
                <parent link="base_link"/>
                <child link="shoulder_link"/>
                <axis xyz="0 0 1"/>
                <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
            </joint>
            
            <joint name="elbow_joint" type="revolute">
                <parent link="shoulder_link"/>
                <child link="elbow_link"/>
                <axis xyz="0 1 0"/>
                <limit lower="-2.0" upper="2.0" effort="80" velocity="1.5"/>
            </joint>
            
            <joint name="wrist_joint" type="revolute">
                <parent link="elbow_link"/>
                <child link="wrist_link"/>
                <axis xyz="1 0 0"/>
                <limit lower="-1.0" upper="1.0" effort="50" velocity="3.0"/>
            </joint>
            
            <joint name="fixed_joint" type="fixed">
                <parent link="base_link"/>
                <child link="hand_link"/>
            </joint>
        </robot>)";
    urdfFile.close();
  }

  void TearDown() override { std::filesystem::remove_all(tempDir_); }

  std::filesystem::path tempDir_;
  std::filesystem::path urdf_path_;
};

// Test constructor with valid URDF
TEST_F(RobotDescriptionTest, Constructor) {
  ASSERT_NO_THROW({ RobotDescription robotDesc(urdf_path_.string()); });

  RobotDescription robotDesc(urdf_path_.string());
  EXPECT_EQ(robotDesc.getURDFPath(), urdf_path_.string());
  EXPECT_EQ(robotDesc.getNumJoints(),
            3);  // Should only count the revolute joints
}

// Test constructor with non-existent URDF
TEST_F(RobotDescriptionTest, ConstructorWithNonexistentFile) {
  std::string nonexistentPath = tempDir_ / "nonexistent.urdf";
  EXPECT_THROW({ RobotDescription robotDesc(nonexistentPath); }, std::runtime_error);
}

// Test constructor with invalid URDF content
TEST_F(RobotDescriptionTest, ConstructorWithInvalidURDF) {
  std::filesystem::path invalidPath = tempDir_ / "invalid.urdf";
  std::ofstream invalidFile(invalidPath);
  invalidFile << "This is not a valid URDF file";
  invalidFile.close();

  EXPECT_THROW({ RobotDescription robotDesc(invalidPath.string()); }, std::runtime_error);
}

// Test getURDFName method
TEST_F(RobotDescriptionTest, GetURDFName) {
  RobotDescription robotDesc(urdf_path_.string());
  EXPECT_EQ(robotDesc.getURDFName(), "test_robot.urdf");
}

// Test containsJoint method
TEST_F(RobotDescriptionTest, ContainsJoint) {
  RobotDescription robotDesc(urdf_path_.string());

  EXPECT_TRUE(robotDesc.containsJoint("shoulder_joint"));
  EXPECT_TRUE(robotDesc.containsJoint("elbow_joint"));
  EXPECT_TRUE(robotDesc.containsJoint("wrist_joint"));
  EXPECT_FALSE(robotDesc.containsJoint("fixed_joint"));
  EXPECT_FALSE(robotDesc.containsJoint("nonexistent_joint"));
}

TEST_F(RobotDescriptionTest, GetJointDescription) {
  RobotDescription robotDesc(urdf_path_.string());

  // Test valid joint
  const JointDescription& shoulderDesc = robotDesc.getJointDescription("shoulder_joint");
  EXPECT_EQ(shoulderDesc.position_bounds.min, -1.57);
  EXPECT_EQ(shoulderDesc.position_bounds.max, 1.57);
  EXPECT_EQ(shoulderDesc.torque_bounds.max, 100.0);
  EXPECT_EQ(shoulderDesc.velocity_bounds.max, 2.0);

  // Test another valid joint
  const JointDescription& elbowDesc = robotDesc.getJointDescription("elbow_joint");
  EXPECT_EQ(elbowDesc.position_bounds.min, -2.0);
  EXPECT_EQ(elbowDesc.position_bounds.max, 2.0);
  EXPECT_EQ(elbowDesc.torque_bounds.max, 80.0);
  EXPECT_EQ(elbowDesc.velocity_bounds.max, 1.5);

  // Test nonexistent joint should be result of MT_DCHECK
  EXPECT_DEATH({ robotDesc.getJointDescription("nonexistent_joint"); }, "");
}

// Test getJointIndex method
TEST_F(RobotDescriptionTest, GetJointIndex) {
  RobotDescription robotDesc(urdf_path_.string());

  int32_t shoulderIndex = robotDesc.getJointIndex("shoulder_joint");
  int32_t elbowIndex = robotDesc.getJointIndex("elbow_joint");
  int32_t wristIndex = robotDesc.getJointIndex("wrist_joint");

  // Each joint should have a unique index
  EXPECT_NE(shoulderIndex, elbowIndex);
  EXPECT_NE(shoulderIndex, wristIndex);
  EXPECT_NE(elbowIndex, wristIndex);

  // Indices should be within expected range (0 to numJoints-1)
  EXPECT_GE(shoulderIndex, 0);
  EXPECT_LT(shoulderIndex, 3);

  // Test nonexistent joint should terminate
  EXPECT_DEATH({ robotDesc.getJointIndex("nonexistent_joint"); }, "");
}

// Test getJointName method
TEST_F(RobotDescriptionTest, GetJointName) {
  RobotDescription robotDesc(urdf_path_.string());

  // Get indices first
  int32_t shoulderIndex = robotDesc.getJointIndex("shoulder_joint");
  int32_t elbowIndex = robotDesc.getJointIndex("elbow_joint");
  int32_t wristIndex = robotDesc.getJointIndex("wrist_joint");

  // Test mapping from index back to name
  EXPECT_EQ(robotDesc.getJointName(shoulderIndex), "shoulder_joint");
  EXPECT_EQ(robotDesc.getJointName(elbowIndex), "elbow_joint");
  EXPECT_EQ(robotDesc.getJointName(wristIndex), "wrist_joint");

  // Test invalid index should terminate
  EXPECT_DEATH({ robotDesc.getJointName(999); }, "");
}

TEST_F(RobotDescriptionTest, JointDescriptionStreamOperator) {
  JointDescription jointDesc;
  jointDesc.name = "test_joint";
  jointDesc.position_bounds = Bounds(-1.5, 1.5);
  jointDesc.velocity_bounds = Bounds(-2.0, 2.0);
  jointDesc.torque_bounds = Bounds(-100.0, 100.0);

  std::stringstream ss;
  ss << jointDesc;

  // This is just a simple check that something was written
  EXPECT_FALSE(ss.str().empty());
}

// Test operator<< for RobotDescription
TEST_F(RobotDescriptionTest, RobotDescriptionStreamOperator) {
  RobotDescription robotDesc(urdf_path_.string());

  std::stringstream ss;
  ss << robotDesc;

  // Expected format may vary depending on your implementation
  // This is just a simple check that something was written
  EXPECT_FALSE(ss.str().empty());

  // Basic checks that the output contains important information
  std::string output = ss.str();
  EXPECT_TRUE(output.find("test_robot.urdf") != std::string::npos);
  EXPECT_TRUE(output.find("shoulder_joint") != std::string::npos);
  EXPECT_TRUE(output.find("elbow_joint") != std::string::npos);
  EXPECT_TRUE(output.find("wrist_joint") != std::string::npos);
}

// Test constructor with vector of JointDescription
TEST_F(RobotDescriptionTest, ConstructorWithVector) {
  std::vector<JointDescription> jointDescs;

  JointDescription j1;
  j1.name = "joint1";
  j1.position_bounds = Bounds(-1.0, 1.0);
  j1.velocity_bounds = Bounds(-1.0, 1.0);
  j1.torque_bounds = Bounds(-10.0, 10.0);
  jointDescs.push_back(j1);

  JointDescription j2;
  j2.name = "joint2";
  j2.position_bounds = Bounds(0.0, 0.5);
  j2.velocity_bounds = Bounds(0.0, 0.5);
  j2.torque_bounds = Bounds(0.0, 20.0);
  jointDescs.push_back(j2);

  RobotDescription robotDesc(jointDescs);

  EXPECT_EQ(robotDesc.getNumJoints(), 2);
  EXPECT_TRUE(robotDesc.containsJoint("joint1"));
  EXPECT_TRUE(robotDesc.containsJoint("joint2"));
  EXPECT_FALSE(robotDesc.containsJoint("joint3"));

  EXPECT_EQ(robotDesc.getJointDescription("joint1").position_bounds.min, -1.0);
}

// Test getJointIndices with vector of names
TEST_F(RobotDescriptionTest, GetJointIndicesVector) {
  RobotDescription robotDesc(urdf_path_.string());

  std::vector<std::string> names = {"shoulder_joint", "wrist_joint"};
  std::vector<joint_index_t> indices = robotDesc.getJointIndices(names);

  EXPECT_EQ(indices.size(), 2);
  EXPECT_EQ(indices[0], robotDesc.getJointIndex("shoulder_joint"));
  EXPECT_EQ(indices[1], robotDesc.getJointIndex("wrist_joint"));
}

// Test JointDescription validation
TEST_F(RobotDescriptionTest, JointDescriptionValidation) {
  JointDescription j;
  j.name = "test";
  j.position_bounds = Bounds(1.0, -1.0);  // Invalid: min > max

  EXPECT_THROW(j.validate(), std::invalid_argument);

  j.position_bounds = Bounds(-1.0, 1.0);  // Valid
  j.velocity_bounds = Bounds(1.0, -1.0);  // Invalid
  EXPECT_THROW(j.validate(), std::invalid_argument);

  j.velocity_bounds = Bounds(-1.0, 1.0);  // Valid
  j.torque_bounds = Bounds(1.0, -1.0);    // Invalid
  EXPECT_THROW(j.validate(), std::invalid_argument);

  j.torque_bounds = Bounds(-1.0, 1.0);  // Valid
  j.validate();                         // Should not throw
}

}  // namespace motorium::model::testing

// Main function that runs all tests
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}