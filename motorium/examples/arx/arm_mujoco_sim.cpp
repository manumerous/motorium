// arm_mujoco_sim.cpp
// Entry-point for the ARX-5 arm MuJoCo simulation.
//
// Usage (Bazel):
//   bazel run //motorium/examples/arx:arm_mujoco_sim

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "motorium_model/RobotDescription.h"
#include "motorium_model/RobotState.h"
#include "motorium_mujoco/MujocoSimInterface.h"

// ---------------------------------------------------------------------------
// ARX-5 joint definitions
// ---------------------------------------------------------------------------
// Bounds are intentionally wide — the XML actuator limits already constrain
// the actual motion; these are only used for RobotDescription bookkeeping.
static const std::vector<motorium::model::JointDescription> kArmJoints = {
    {"joint1", {-10.0, 10.0}, {-10.0, 10.0}, {-100.0, 100.0}},
    {"joint2", {-10.0, 10.0}, {-10.0, 10.0}, {-100.0, 100.0}},
    {"joint3", {-10.0, 10.0}, {-10.0, 10.0}, {-100.0, 100.0}},
    {"joint4", {-10.0, 10.0}, {-10.0, 10.0}, {-100.0, 100.0}},
    {"joint5", {-10.0, 10.0}, {-10.0, 10.0}, {-100.0, 100.0}},
    {"joint6", {-5.0, 5.0},   {-10.0, 10.0}, {-100.0, 100.0}},
    {"gripper_left_joint",  {0.0, 0.046}, {-10.0, 10.0}, {-3.0, 3.0}},
    {"gripper_right_joint", {0.0, 0.046}, {-10.0, 10.0}, {-3.0, 3.0}},
};

int main(int argc, char** argv) {
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
  config.scenePath       = scene_path;
  config.initStatePtr_   = nullptr;   // zero init; arm starts at rest
  config.dt              = 0.001;     // 1 kHz physics
  config.renderFrequencyHz = 60.0;
  config.headless        = false;
  config.verbose         = true;

  // ── Create and start simulation ────────────────────────────────────────
  std::cout << "[arm_mujoco_sim] Starting simulation...\n";
  motorium::mujoco::MujocoSimInterface sim(config, robot_description);
  sim.initSim();
  sim.start();

  std::cout << "[arm_mujoco_sim] Simulation running. Close the viewer window to exit.\n";

  // Block the main thread; the renderer and physics run on their own threads.
  // Press ESC in the MuJoCo viewer to close the window.
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  sim.stop();
  return 0;
}
