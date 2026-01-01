#include <motorium_mujoco/MujocoSimInterface.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

/**
 * @brief Initializes a Mujoco simulation for the g1 robot and runs a continuous controller loop.
 *
 * Attempts to resolve URDF and MJX files from the specified package (defaults to "g1_description"),
 * configures and starts the MujocoSimInterface, and enters an infinite loop that updates the interface
 * state and applies fixed joint action gains.
 *
 * @throws std::runtime_error If the package share directory for the specified model folder cannot be resolved.
 * @returns int 0 on successful termination (normally unreachable because the function enters an infinite loop).
 */
int main(int argc, char* argv[]) {
  // Default value in case no argument is provided
  std::string model_folder = "g1_description";

  // Check if at least one argument (besides the program name) was provided
  if (argc > 1) {
    model_folder = argv[1];
  } else {
    std::cerr << "Warning: No model folder specified. Using default: " << model_folder << std::endl;
  }

  std::string urdfFile;
  try {
    urdfFile = ament_index_cpp::get_package_share_directory(model_folder) + "/urdf/g1_29dof.urdf";
  } catch (const std::exception& e) {
    throw std::runtime_error("Failed to get package share directory: g1_description. Error: " + std::string(e.what()));
  }

  std::string mjxFile;
  try {
    mjxFile = ament_index_cpp::get_package_share_directory(model_folder) + "/urdf/g1_29dof.xml";
  } catch (const std::exception& e) {
    throw std::runtime_error("Failed to get package share directory: g1_description. Error: " + std::string(e.what()));
  }

  motorium::model::RobotState initState(motorium::model::RobotDescription(urdfFile), 2);
  initState.setConfigurationToZero();
  initState.setRootPositionInWorldFrame(motorium::vector3_t(0.0, 0.0, 0.85));

  motorium::mujoco::MujocoSimConfig config;
  config.dt = 0.001;

  config.scenePath = mjxFile;
  config.verbose = true;

  motorium::mujoco::MujocoSimInterface robotInterface(config, urdfFile);
  robotInterface.start();

  // Simulated controller loop;
  while (true) {
    robotInterface.updateInterfaceStateFromRobot();
    auto actions = robotInterface.getRobotJointAction();
    for (auto action : actions) {
      action.kp = 200;
      action.kd = 2.0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  return 0;
}