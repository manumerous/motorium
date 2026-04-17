// arm_mujoco_sim.cpp
// Entry-point for the ARX-5 arm MuJoCo simulation.
//
// Usage (Bazel):
//   bazel run //motorium/examples/arx:arm_mujoco_sim

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "tools/cpp/runfiles/runfiles.h"

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
    {"joint5", {-10.0, 10.0}, {-10.0, 10.0}, {-100.0, 100.0}}, {"joint6", {-5.0, 5.0}, {-10.0, 10.0}, {-100.0, 100.0}}};

int main(int argc, char** argv) {
  // ── Scene path ─────────────────────────────────────────────────────────
  std::string scene_path;
  if (argc > 1) {
    scene_path = argv[1];
  } else {
    std::string error;
    auto runfiles = bazel::tools::cpp::runfiles::Runfiles::Create(argv[0], &error);
    if (!runfiles) {
      std::cerr << "[arm_mujoco_sim] ERROR: Could not initialize runfiles: " << error << "\n"
                << "  Pass the scene path as argv[1] to override.\n";
      return 1;
    }
    scene_path = runfiles->Rlocation("_main/motorium/examples/arx/model/arx_arm_scene.xml");
    if (!std::filesystem::exists(scene_path)) {
      std::cerr << "[arm_mujoco_sim] ERROR: arx_arm_scene.xml not found via runfiles at: " << scene_path << "\n"
                << "  Pass the scene path as argv[1] to override.\n";
      return 1;
    }
  }

  std::cout << "[arm_mujoco_sim] Loading scene: " << scene_path << "\n";

  // ── Robot description ──────────────────────────────────────────────────
  motorium::model::RobotDescription robot_description(kArmJoints);

  // ── Sim config ─────────────────────────────────────────────────────────
  motorium::mujoco::MujocoSimConfig config;
  config.scenePath = scene_path;
  config.initStatePtr_ = nullptr;  // zero init; arm starts at rest
  config.dt = 0.0002;              // 5 kHz physics
  config.renderFrequencyHz = 60.0;
  config.headless = false;
  config.verbose = true;

  // ── Create and start simulation ────────────────────────────────────────
  std::cout << "[arm_mujoco_sim] Starting simulation...\n";
  motorium::mujoco::MujocoSimInterface sim(config, robot_description);

  motorium::model::RobotJointFeedbackAction action(robot_description);

  // Helper: set gains for a joint. kd = 2 * dampratio * sqrt(kp)
  auto setJointGains = [&](const std::string& name, double q_des, double v_des, double kp, double dampratio, double ff = 0.0) {
    auto& a = action.at(robot_description.getJointIndex(name));
    a.q_des = q_des;
    a.v_des = v_des;
    a.kp = kp;
    a.kd = 2.0 * dampratio * std::sqrt(kp);
    a.feed_forward_effort = ff;
  };

  //                     name       q_des        v_des       kp     dampratio
  // setJointGains("joint1", 3.14, 0.0, 50.0, 0.7);
  // setJointGains("joint2", 0.0, 0.0, 250.0, 0.9);
  // setJointGains("joint3", 0.0, 0.0, 350.0, 0.8);
  // setJointGains("joint4", 0.0, 0.0, 100.0, 1.1);
  // setJointGains("joint5", 0.0, 0.0, 10.0, 0.7);
  // setJointGains("joint6", 0.0, 0.0, 5.0, 1.4);

  setJointGains("joint1", 0.0, 0.0, 200.0, 0.7);
  setJointGains("joint2", 0.0, 0.0, 250.0, 1.1);
  setJointGains("joint3", 0.0, 0.0, 250.0, 1.1);
  setJointGains("joint4", 0.0, 0.0, 100.0, 1.1);
  setJointGains("joint5", 0.0, 0.0, 10.0, 0.7);
  setJointGains("joint6", 0.0, 0.0, 5.0, 1.4);

  sim.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  std::cout << "[arm_mujoco_sim] Simulation running. Close the viewer window to exit.\n";

  motorium::model::RobotState state(robot_description);

  // Block the main thread; the renderer and physics run on their own threads.
  // Press ESC in the MuJoCo viewer to close the window.
  while (true) {
    sim.updateRobotState(state);
    auto& joint1_action = action.at(robot_description.getJointIndex("joint2"));
    auto& joint2_action = action.at(robot_description.getJointIndex("joint3"));
    joint1_action.q_des = sin(2.0 * M_PI * state.getTime()) + 0.5;
    joint2_action.q_des = sin(2.0 * M_PI * state.getTime()) + 0.5;
    sim.setJointFeedbackAction(action);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  sim.stop();
  return 0;
}
