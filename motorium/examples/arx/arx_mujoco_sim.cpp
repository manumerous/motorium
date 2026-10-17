// arm_mujoco_sim.cpp
// Entry-point for the ARX-5 arm MuJoCo simulation.
//
// Usage (Bazel):
//   bazel run //motorium/examples/arx:arm_mujoco_sim

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "motorium_control/JointPDController.h"
#include "motorium_model/RobotDescription.h"
#include "motorium_model/RobotJointFeedbackAction.h"
#include "motorium_model/RobotState.h"
#include "motorium_mujoco/MujocoSimInterface.h"

// ---------------------------------------------------------------------------
// ARX-5 joint definitions
// ---------------------------------------------------------------------------
// Bounds are intentionally wide — the XML actuator limits already constrain
// the actual motion; these are only used for RobotDescription bookkeeping.
static const std::vector<motorium::model::JointDescription> kArmJoints = {
    {"joint1", {-10.0, 10.0}, {-10.0, 10.0}, {-100.0, 100.0}}, {"joint2", {-10.0, 10.0}, {-10.0, 10.0}, {-100.0, 100.0}},
    {"joint3", {-10.0, 10.0}, {-10.0, 10.0}, {-100.0, 100.0}}, {"joint4", {-10.0, 10.0}, {-10.0, 10.0}, {-100.0, 100.0}},
    {"joint5", {-10.0, 10.0}, {-10.0, 10.0}, {-100.0, 100.0}}, {"joint6", {-5.0, 5.0}, {-10.0, 10.0}, {-100.0, 100.0}},
    // {"gripper_left_joint", {0.0, 0.046}, {-10.0, 10.0}, {-3.0, 3.0}}, {"gripper_right_joint", {0.0, 0.046}, {-10.0, 10.0}, {-3.0, 3.0}},
};

// ---------------------------------------------------------------------------
// PD controller parameters
// ---------------------------------------------------------------------------
static const std::vector<std::string> kControlledJoints = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

static constexpr int kNumControlledJoints = 6;

// Sinusoidal setpoint parameters
static constexpr double kAmplitudeRad = M_PI / 20.0;  // 90 deg
static constexpr double kFrequencyHz = 0.25;          // 0.25 Hz  → 4 s period
static constexpr double kOmega = 2.0 * M_PI * kFrequencyHz;

int main(int argc, char** argv) {
  for (const motorium::model::JointDescription& joint : kArmJoints) {
    std::cout << joint << "\n";
  }

  // ── Scene path ─────────────────────────────────────────────────────────
  // Default to the Bazel runfiles-relative path; override via argv[1].
  std::string scene_path = "/home/manu/src/motorium_ws/src/motorium/motorium/examples/arx/model/arx_arm_scene.xml";
  if (argc > 1) {
    scene_path = argv[1];
  }

  std::cout << "[arm_mujoco_sim] Loading scene: " << scene_path << "\n";

  // ── Robot description ──────────────────────────────────────────────────
  motorium::model::RobotDescription robot_description(kArmJoints);

  // ── Sim config ─────────────────────────────────────────────────────────
  motorium::mujoco::MujocoSimConfig config;
  config.scenePath = scene_path;
  config.initStatePtr_ = nullptr;  // zero init; arm starts at rest
  config.dt = 0.001;               // 1 kHz physics
  config.renderFrequencyHz = 60.0;
  config.headless = false;
  config.verbose = true;

  // ── Create and start simulation ────────────────────────────────────────
  std::cout << "[arm_mujoco_sim] Starting simulation...\n";
  motorium::mujoco::MujocoSimInterface sim(config, robot_description);
  sim.initSim();
  sim.start();

  // ── Set up ExplicitJointPDController ───────────────────────────────────
  motorium::control::JointPDControllerConfig pd_config;
  pd_config.joint_names = kControlledJoints;

  motorium::vector_t kp(kNumControlledJoints);
  motorium::vector_t kd(kNumControlledJoints);
  kp << 10.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  kd << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  pd_config.kp = kp;
  pd_config.kd = kd;

  motorium::control::ExplicitJointPDController controller(robot_description, pd_config);

  // ── State / action buffers ─────────────────────────────────────────────
  motorium::model::RobotState current_state(robot_description);
  motorium::model::RobotState desired_state(robot_description);
  motorium::model::RobotJointFeedbackAction action(robot_description);

  std::cout << "[arm_mujoco_sim] Simulation running with extrinsic PD control.\n"
            << "  Sinusoidal setpoint: amplitude = 90 deg, frequency = " << kFrequencyHz << " Hz\n"
            << "  Close the viewer window to exit.\n";

  // ── Control loop (~1 kHz) ──────────────────────────────────────────────
  const auto t0 = std::chrono::steady_clock::now();

  motorium::joint_index_t joint_1_index = robot_description.getJointIndex("joint1");

  while (true) {
    // Wall-clock elapsed time in seconds
    const auto now = std::chrono::steady_clock::now();
    const double t = std::chrono::duration<double>(now - t0).count();

    // Sinusoidal position & velocity setpoints
    const double q_des = kAmplitudeRad * std::sin(kOmega * t);

    // Fill desired state for every controlled joint

    motorium::model::JointState js;
    js.q = q_des;
    // std::cout << "[arm_mujoco_sim] Desired position: " << js.q << "\n";
    js.v = 0.0;
    js.effort = 0.0;
    desired_state.setJointState(joint_1_index, js);

    // Read current sim state
    sim.updateRobotState(current_state);
    // std::cout << joint_1_index << "\n";
    // std::cout << "[arm_mujoco_sim] Current position: " << current_state.getJointState(joint_1_index).q << "\n";

    // std::cout << robot_description << std::endl;
    // Compute explicit PD control action
    controller.computeJointControlAction(t, current_state, desired_state, action);

    // Apply control action to simulation
    sim.setJointFeedbackAction(action);

    // Sleep ~1 ms to yield CPU (physics runs on its own thread at 1 kHz)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  sim.stop();
  return 0;
}
