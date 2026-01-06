#include <gtest/gtest.h>
#include <motorium_model/RobotState.h>

#include <filesystem>
#include <fstream>
#include <vector>

namespace motorium::model {
namespace testing {

class RobotStateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tempDir_ = std::filesystem::temp_directory_path() / "robot_state_test";
    std::filesystem::create_directories(tempDir_);
    urdf_path_ = tempDir_ / "test_robot_state.urdf";

    // Create a simple test URDF
    std::ofstream urdfFile(urdf_path_);
    urdfFile << R"(<?xml version="1.0"?>
        <robot name="test_robot">
            <link name="base_link"/>
            <link name="link1"/>
            <joint name="joint1" type="revolute">
                <parent link="base_link"/>
                <child link="link1"/>
                <axis xyz="0 0 1"/>
                <limit lower="-1.0" upper="1.0" effort="10.0" velocity="1.0"/>
            </joint>
            <link name="link2"/>
            <joint name="joint2" type="prismatic">
                <parent link="link1"/>
                <child link="link2"/>
                <axis xyz="1 0 0"/>
                <limit lower="0.0" upper="0.5" effort="20.0" velocity="0.5"/>
            </joint>
        </robot>)";
    urdfFile.close();

    robot_desc_ = std::make_unique<RobotDescription>(urdf_path_.string());
  }

  void TearDown() override { std::filesystem::remove_all(tempDir_); }

  std::filesystem::path tempDir_;
  std::filesystem::path urdf_path_;
  std::unique_ptr<RobotDescription> robot_desc_;
};

TEST_F(RobotStateTest, ConstructorAndInitialization) {
  RobotState state(*robot_desc_);

  // check time is 0
  EXPECT_DOUBLE_EQ(state.getTime(), 0.0);

  // Check contact flags default (size 1, true)
  EXPECT_EQ(state.getContactFlags().size(), 1);
  EXPECT_TRUE(state.getContactFlag(0));

  // Check root state is zero/identity
  EXPECT_TRUE(state.getRootPositionInWorldFrame().isZero());
  EXPECT_TRUE(state.getRootLinearVelocityInLocalFrame().isZero());
  EXPECT_TRUE(state.getRootAngularVelocityInLocalFrame().isZero());
  EXPECT_TRUE(state.getRootRotationLocalToWorldFrame().isApprox(quaternion_t::Identity()));

  // Check joints are zero
  EXPECT_DOUBLE_EQ(state.getJointPosition(0), 0.0);  // joint1 index should be 0 or 1
  EXPECT_DOUBLE_EQ(state.getJointVelocity(0), 0.0);
}

TEST_F(RobotStateTest, SetConfigurationToZero) {
  RobotState state(*robot_desc_);

  // Modify state
  state.setTime(1.5);
  state.setRootPositionInWorldFrame(vector3_t(1, 2, 3));
  state.setJointPosition(0, 0.5);
  state.setContactFlag(0, false);

  // Verify modification
  EXPECT_DOUBLE_EQ(state.getTime(), 1.5);
  EXPECT_FALSE(state.getRootPositionInWorldFrame().isZero());
  EXPECT_DOUBLE_EQ(state.getJointPosition(0), 0.5);
  EXPECT_FALSE(state.getContactFlag(0));

  // Reset
  state.setConfigurationToZero();

  // Verify reset
  // Time is NOT reset by setConfigurationToZero based on code reading (only root and joints and contacts)
  EXPECT_DOUBLE_EQ(state.getTime(), 1.5);
  EXPECT_TRUE(state.getRootPositionInWorldFrame().isZero());
  EXPECT_DOUBLE_EQ(state.getJointPosition(0), 0.0);
  EXPECT_TRUE(state.getContactFlag(0));  // Resets to true
}

TEST_F(RobotStateTest, RootStateAccessors) {
  RobotState state(*robot_desc_);

  vector3_t pos(1.0, 2.0, 3.0);
  state.setRootPositionInWorldFrame(pos);
  EXPECT_TRUE(state.getRootPositionInWorldFrame().isApprox(pos));

  quaternion_t rot(0.707, 0, 0.707, 0);
  rot.normalize();
  state.setRootRotationLocalToWorldFrame(rot);
  EXPECT_TRUE(state.getRootRotationLocalToWorldFrame().isApprox(rot));

  vector3_t lin_vel(0.1, 0.2, 0.3);
  state.setRootLinearVelocityInLocalFrame(lin_vel);
  EXPECT_TRUE(state.getRootLinearVelocityInLocalFrame().isApprox(lin_vel));

  vector3_t ang_vel(0.01, 0.02, 0.03);
  state.setRootAngularVelocityInLocalFrame(ang_vel);
  EXPECT_TRUE(state.getRootAngularVelocityInLocalFrame().isApprox(ang_vel));
}

TEST_F(RobotStateTest, JointStateAccessors) {
  RobotState state(*robot_desc_);

  auto joint1_idx = robot_desc_->getJointIndex("joint1");
  auto joint2_idx = robot_desc_->getJointIndex("joint2");

  state.setJointPosition(joint1_idx, 0.5);
  state.setJointVelocity(joint1_idx, 0.1);

  EXPECT_DOUBLE_EQ(state.getJointPosition(joint1_idx), 0.5);
  EXPECT_DOUBLE_EQ(state.getJointVelocity(joint1_idx), 0.1);

  // Test full struct setter
  JointState js;
  js.position = 0.8;
  js.velocity = 0.2;
  js.measuredEffort = 5.0;
  state.setJointState(joint2_idx, js);

  auto ret_js = state.getJointState(joint2_idx);
  EXPECT_DOUBLE_EQ(ret_js.position, 0.8);
  EXPECT_DOUBLE_EQ(ret_js.velocity, 0.2);
  EXPECT_DOUBLE_EQ(ret_js.measuredEffort, 5.0);
}

TEST_F(RobotStateTest, VectorAccessors) {
  RobotState state(*robot_desc_);

  auto joint1_idx = robot_desc_->getJointIndex("joint1");
  auto joint2_idx = robot_desc_->getJointIndex("joint2");

  state.setJointPosition(joint1_idx, 0.5);
  state.setJointPosition(joint2_idx, 0.25);
  state.setJointVelocity(joint1_idx, 1.0);
  state.setJointVelocity(joint2_idx, 0.5);

  std::vector<joint_index_t> ids = {joint1_idx, joint2_idx};

  vector_t positions = state.getJointPositions(ids);
  EXPECT_EQ(positions.size(), 2);
  EXPECT_DOUBLE_EQ(positions[0], 0.5);
  EXPECT_DOUBLE_EQ(positions[1], 0.25);

  vector_t velocities = state.getJointVelocities(ids);
  EXPECT_EQ(velocities.size(), 2);
  EXPECT_DOUBLE_EQ(velocities[0], 1.0);
  EXPECT_DOUBLE_EQ(velocities[1], 0.5);
}

TEST_F(RobotStateTest, ContactFlags) {
  RobotState state(*robot_desc_, 2);  // 2 contacts

  EXPECT_EQ(state.getContactFlags().size(), 2);
  EXPECT_TRUE(state.getContactFlag(0));
  EXPECT_TRUE(state.getContactFlag(1));

  state.setContactFlag(1, false);
  EXPECT_FALSE(state.getContactFlag(1));
  EXPECT_TRUE(state.getContactFlag(0));
}

}  // namespace testing
}  // namespace motorium::model

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
