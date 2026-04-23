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

#include "motorium_control/JointPDController.h"
#include "tools/cpp/runfiles/runfiles.h"

#include "motorium_model/RobotDescription.h"
#include "motorium_model/RobotJointFeedbackAction.h"
#include "motorium_model/RobotState.h"
#include "motorium_mujoco/MujocoDriver.h"

using namespace motorium;

// ---------------------------------------------------------------------------
// ARX-5 joint definitions
// ---------------------------------------------------------------------------
// Bounds are intentionally wide — the XML actuator limits already constrain
// the actual motion; these are only used for RobotDescription bookkeeping.
static const std::vector<model::JointDescription> kArmJoints = {
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
  model::RobotDescription robot_description(kArmJoints);

  // ── Sim config ─────────────────────────────────────────────────────────
  mujoco::MujocoSimConfig config;
  config.scenePath = scene_path;
  config.initStatePtr_ = nullptr;  // zero init; arm starts at rest
  config.dt = 0.0002;              // 5 kHz physics
  config.renderFrequencyHz = 60.0;
  config.headless = false;
  config.verbose = true;

  // ── Create and start simulation ────────────────────────────────────────
  std::cout << "[arm_mujoco_sim] Starting simulation...\n";
  mujoco::MujocoDriver sim(config, robot_description);

  model::RobotJointFeedbackAction action(robot_description);

  control::ImplicitJointPDController::Config controller_config{
      {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"},
      {200.0, 250.0, 250.0, 100.0, 10.0, 5.0},  // kp
      {19.8, 34.6, 34.6, 21.9, 4.4, 6.3},       // kd = 2*dampratio*sqrt(kp)
  };

  control::ImplicitJointPDController controller(robot_description, controller_config);

  sim.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  std::cout << "[arm_mujoco_sim] Simulation running. Close the viewer window to exit.\n";

  model::RobotState state(robot_description);
  model::RobotState desired_state(robot_description);

  // Block the main thread; the renderer and physics run on their own threads.
  // Press ESC in the MuJoCo viewer to close the window.
  while (true) {
    sim.update(action, state);
    controller.computeJointControlAction(state.getTime(), state, desired_state, action);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  sim.stop();
  return 0;
}
