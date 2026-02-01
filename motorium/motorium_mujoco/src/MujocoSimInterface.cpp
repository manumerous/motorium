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

#include "motorium_mujoco/MujocoSimInterface.h"

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace motorium::mujoco {

MjState::MjState(const mjModel* mj_model) : data(mj_makeData(mj_model)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

MujocoSimInterface::MujocoSimInterface(const MujocoSimConfig& config, const model::RobotDescription& robot_description)
    : DriverBase(robot_description, "mujoco_sim"),
      config_(config),
      action_internal_(robot_description),
      headless_(config.headless),
      verbose_(config.verbose) {
  last_realtime_ = std::chrono::high_resolution_clock::now();
  const int errstr_sz = 1000;  // Define the size of the error buffer
  char errstr[errstr_sz];      // Declare the error string buffer

  // option 1: parse and compile XML from file
  mj_model_ = mj_loadXML(config.scenePath.c_str(), NULL, errstr, errstr_sz);
  if (!mj_model_) {
    std::cerr << "Could not load MuJoCo model: " << config.scenePath << ". Error: " << std::strerror(errno) << std::endl;
    throw std::runtime_error("Could not load MuJoCo: " + std::string(std::strerror(errno)));
  }

  // Create data
  mj_data_ = mj_makeData(mj_model_);

  /* initialize random seed: */
  srand(time(NULL));

  mj_contact_ = mj_data_->contact;

  // assert(num_active_joints_ == neo_definitions::FULL_NEO_JOINT_DIM);

  mj_model_->opt.timestep = config_.dt;

  time_step_micro_ = static_cast<size_t>(config_.dt * 1000000);

  if (verbose_) printModelInfo();

  setupJointIndexMaps(robot_description);

  model::RobotState initRobotState(robot_description, 2);

  if (config_.initStatePtr_ != nullptr) {
    initRobotState = *config.initStatePtr_;
  } else {
    initRobotState.setConfigurationToZero();
    initRobotState.setRootPositionInWorldFrame(vector3_t(0.0, 0.0, 1.0));
  }
  setSimState(initRobotState);

  // Add default joint damping
  scalar_t defaultJointDamping = 10.0;

  for (int i = 6; i < mj_model_->nv; ++i) {
    std::string mjJointName(&mj_model_->names[mj_model_->name_jntadr[mj_model_->dof_jntid[i]]]);
    std::cerr << "mjJointName: " << mjJointName << std::endl;
    mj_model_->dof_damping[i] = defaultJointDamping;
  }

  for (int i = 0; i < mj_model_->nsensor; i++) {
    std::string sensorName(&mj_model_->names[mj_model_->name_sensoradr[i]]);

    if (sensorName == "right_foot_touch_sensor") {
      right_foot_touch_sensor_addr_ = mj_model_->sensor_adr[i];
    }
    if (sensorName == "left_foot_touch_sensor") {
      left_foot_touch_sensor_addr_ = mj_model_->sensor_adr[i];
    }
    if (sensorName == "right_foot_force_sensor") {
      right_foot_sensor_addr_ = mj_model_->sensor_adr[i];
    }
    if (sensorName == "left_foot_force_sensor") {
      left_foot_sensor_addr_ = mj_model_->sensor_adr[i];
    }
  }

  qpos_init_ = new mjtNum[mj_model_->nq];
  qvel_init_ = new mjtNum[mj_model_->nv];

  // Safe init state for resets
  memcpy(qpos_init_, mj_data_->qpos, mj_model_->nq * sizeof(mjtNum));
  memcpy(qvel_init_, mj_data_->qvel, mj_model_->nv * sizeof(mjtNum));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

MujocoSimInterface::~MujocoSimInterface() {
  stop();
  mj_deleteData(mj_data_);
  mj_deleteModel(mj_model_);
  delete[] qpos_init_;
  delete[] qvel_init_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void MujocoSimInterface::reset() {
  memcpy(mj_data_->qpos, qpos_init_, mj_model_->nq * sizeof(mjtNum));
  memcpy(mj_data_->qvel, qvel_init_, mj_model_->nv * sizeof(mjtNum));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void MujocoSimInterface::copyMjState(MjState& state) const {
  {
    std::lock_guard<std::mutex> guard(mj_mutex_);

    state.timestamp = mj_data_->time;
    mj_copyData(state.data, mj_model_, mj_data_);

    state.metrics = metrics_;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void MujocoSimInterface::setupJointIndexMaps(const model::RobotDescription& robot_description) {
  // Mujoco to Robot joints
  for (int i = 1; i < mj_model_->njnt; ++i) {
    // Get the joint name
    const std::string jointName(&mj_model_->names[mj_model_->name_jntadr[i]]);
    if (robot_description.containsJoint(jointName)) {
      active_joint_names_.emplace_back(jointName);
    } else {
      std::cerr << "WARNING: Joint contained in mujoco xml not exposed to "
                   "RobotHWInterface: "
                << jointName << std::endl;
    }
  }

  active_robot_joint_indices_ = robot_description.getJointIndices(active_joint_names_);

  // Mujoco to robot actuators
  for (int i = 0; i < mj_model_->nu; ++i) {
    const std::string actuator_name = mj_id2name(mj_model_, mjOBJ_ACTUATOR, i);

    if (robot_description.containsJoint(actuator_name)) {
      active_actuator_names_.emplace_back(actuator_name);
    } else {
      std::cerr << "WARNING: Actuator contained in mujoco xml not be commanded "
                   "through RobotHWInterface: "
                << actuator_name << std::endl;
    }
  }

  active_robot_actuator_indices_ = robot_description.getJointIndices(active_actuator_names_);

  num_active_joints_ = active_robot_joint_indices_.size();
  num_actuators_ = active_robot_actuator_indices_.size();
  if (verbose_) {
    std::cerr << "Initialized " << num_active_joints_ << " active Joints" << std::endl;
    std::cerr << "Initialized " << num_actuators_ << " active Actuators" << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void MujocoSimInterface::printModelInfo() {
  std::cerr << "time_step_micro_: " << time_step_micro_ << std::endl;

  std::cerr << "njnt: " << mj_model_->njnt << std::endl;
  std::cerr << "nq: " << mj_model_->nq << std::endl;
  std::cerr << "nv: " << mj_model_->nv << std::endl;
  std::cerr << "nu: " << mj_model_->nu << std::endl;

  for (int i = 0; i < mj_model_->nbody; ++i) {
    std::string bodyName(&mj_model_->names[mj_model_->name_bodyadr[i]]);
    std::cerr << "Body " << i << ": " << bodyName << std::endl;

    std::cerr << "  Position: ";
    for (size_t j = 0; j < 3; ++j) {
      std::cerr << mj_data_->xpos[i * 3 + j] << " ";
    }
    std::cerr << std::endl;

    // Print orientation quaternion
    std::cerr << "  Orientation (Quaternion): ";
    for (size_t j = 0; j < 4; ++j) {
      std::cerr << mj_data_->xquat[i * 4 + j] << " ";
    }
    std::cerr << std::endl;
  }

  std::string jointName(&mj_model_->names[mj_model_->name_jntadr[0]]);

  // Print the information
  std::cerr << "Joint Name: " << jointName << std::endl;
  std::cerr << "Position: " << mj_data_->qpos[0] << " " << mj_data_->qpos[1] << " " << mj_data_->qpos[2] << " " << mj_data_->qpos[3] << " "
            << mj_data_->qpos[4] << " " << mj_data_->qpos[5] << " " << mj_data_->qpos[6] << std::endl;
  std::cerr << "Velocity: " << mj_data_->qvel[0] << " " << mj_data_->qvel[1] << " " << mj_data_->qvel[2] << " " << mj_data_->qvel[3] << " "
            << mj_data_->qvel[4] << " " << mj_data_->qvel[5] << std::endl;

  // Print joint names, positions, and velocities
  for (int i = 1; i < mj_model_->njnt; ++i) {
    // Get the joint name
    std::string jointName(&mj_model_->names[mj_model_->name_jntadr[i]]);

    // Get the joint position and velocity
    double jointPos = mj_data_->qpos[i + 6];
    double jointVel = mj_data_->qvel[i + 5];

    // Print the information
    std::cerr << "Joint Name: " << jointName << ", Position: " << jointPos << ", Velocity: " << jointVel << std::endl;
  }

  // Calculate total mass
  scalar_t totalMass = 0.0;
  for (int i = 0; i < mj_model_->nbody; i++) {
    totalMass += mj_model_->body_mass[i];
  }
  std::cerr << "Total MuJoCo model mass: " << totalMass << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void MujocoSimInterface::setSimState(const model::RobotState& robot_state) {
  // Root Pose
  vector3_t rootPosition = robot_state.getRootPositionInWorldFrame();
  quaternion_t quat_l_to_w = robot_state.getRootRotationLocalToWorldFrame();

  mj_data_->qpos[0] = rootPosition[0];
  mj_data_->qpos[1] = rootPosition[1];
  mj_data_->qpos[2] = rootPosition[2];
  mj_data_->qpos[3] = quat_l_to_w.w();
  mj_data_->qpos[4] = quat_l_to_w.x();
  mj_data_->qpos[5] = quat_l_to_w.y();
  mj_data_->qpos[6] = quat_l_to_w.z();

  // Root Velocity

  vector3_t root_vel_lin_world_frame = quat_l_to_w * robot_state.getRootLinearVelocityInLocalFrame();
  vector3_t root_vel_ang_local_frame = robot_state.getRootAngularVelocityInLocalFrame();

  mj_data_->qvel[0] = root_vel_lin_world_frame[0];
  mj_data_->qvel[1] = root_vel_lin_world_frame[1];
  mj_data_->qvel[2] = root_vel_lin_world_frame[2];
  mj_data_->qvel[3] = root_vel_ang_local_frame[0];
  mj_data_->qvel[4] = root_vel_ang_local_frame[1];
  mj_data_->qvel[5] = root_vel_ang_local_frame[2];

  // Joint State
  for (size_t i = 0; i < num_active_joints_; ++i) {
    mj_data_->qpos[i + 7] = robot_state.getJointPosition(active_robot_joint_indices_[i]);
    mj_data_->qvel[i + 6] = robot_state.getJointVelocity(active_robot_joint_indices_[i]);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void MujocoSimInterface::updateRobotState(model::RobotState& robot_state) {
  std::lock_guard<std::mutex> lock(mj_mutex_);
  // Update mujoco joint angles
  for (size_t i = 0; i < num_active_joints_; ++i) {
    robot_state.setJointPosition(active_robot_joint_indices_[i], mj_data_->qpos[i + 7]);
    robot_state.setJointVelocity(active_robot_joint_indices_[i], mj_data_->qvel[i + 6]);
  }

  // Initialize in order w, x,y ,z
  quaternion_t quat_l_to_w = quaternion_t(mj_data_->qpos[3], mj_data_->qpos[4], mj_data_->qpos[5], mj_data_->qpos[6]);
  vector3_t pelvisAngularVelLocal = vector3_t(mj_data_->qvel[3], mj_data_->qvel[4], mj_data_->qvel[5]);

  // Fix later
  // bool leftFootContact =
  // (mj_data_->sensordata[left_foot_touch_sensor_addr_] > 0.1); bool
  // rightFootContact = (mj_data_->sensordata[right_foot_touch_sensor_addr_]
  // > 0.1);
  bool leftFootContact = true;
  bool rightFootContact = true;

  robot_state.setRootPositionInWorldFrame(vector3_t(mj_data_->qpos[0], mj_data_->qpos[1], mj_data_->qpos[2]));
  robot_state.setRootRotationLocalToWorldFrame(quat_l_to_w);
  // Rotate the angular velocity from world frame to local frame.
  robot_state.setRootLinearVelocityInLocalFrame(quat_l_to_w.inverse() * vector3_t(mj_data_->qvel[0], mj_data_->qvel[1], mj_data_->qvel[2]));
  robot_state.setRootAngularVelocityInLocalFrame(pelvisAngularVelLocal);
  robot_state.setContactFlag(0, leftFootContact);
  robot_state.setContactFlag(1, rightFootContact);

  robot_state.setTime(mj_data_->time);  // Todo: should mujoco be the source of time?
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void MujocoSimInterface::updateMetrics() {
  simFps_.tick();

  metrics_.fps_sim = simFps_.fps();

  auto nowRealTime = std::chrono::high_resolution_clock::now();
  auto realElapsedTime = std::chrono::duration<double>(nowRealTime - last_realtime_).count();
  last_realtime_ = nowRealTime;

  metrics_.drift_tick = config_.dt - realElapsedTime;
  metrics_.drift_cumulative += metrics_.drift_tick;

  metrics_.rtf_tick = config_.dt / realElapsedTime;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void MujocoSimInterface::setJointFeedbackAction(const model::RobotJointFeedbackAction& action) {
  std::lock_guard<std::mutex> lock(action_mutex_);
  action_internal_ = action;
}

void MujocoSimInterface::simulationStep() {
  {
    std::lock_guard<std::mutex> lock(action_mutex_);

    for (size_t i = 0; i < num_actuators_; ++i) {
      joint_index_t idx = active_robot_actuator_indices_[i];
      const motorium::model::JointFeedbackAction& JointFeedbackAction = action_internal_.at(idx);
      mj_data_->ctrl[i] = JointFeedbackAction.getTotalFeedbackTorque(mj_data_->qpos[i + 7], mj_data_->qvel[i + 6]);
    }
  }
  {
    std::lock_guard<std::mutex> lock(mj_mutex_);
    mj_step(mj_model_, mj_data_);
    updateThreadSafeRobotState();
    updateMetrics();

    // Auto reset logic.
    if (mj_data_->qpos[2] < 0.2) {
      reset();
      for (size_t i = 0; i < num_actuators_; ++i) {
        mj_data_->ctrl[i] = 0.0;
      }
      mj_step(mj_model_, mj_data_);
      updateThreadSafeRobotState();
      simFps_.reset();
      metrics_.reset();
      updateMetrics();
      // Sleep to let controller update and adjust;
      std::this_thread::sleep_until(std::chrono::steady_clock::now() + std::chrono::microseconds(1000000));
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void MujocoSimInterface::simulationLoop(std::stop_token st) {
  simFps_.reset();
  metrics_.reset();
  auto nextWakeup = std::chrono::steady_clock::now();
  while (!st.stop_requested()) {
    simulationStep();

    // Sleep in case sim loop is faster than specified sim rate.
    nextWakeup += std::chrono::microseconds(time_step_micro_);
    std::this_thread::sleep_until(nextWakeup);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void MujocoSimInterface::initSim() {
  simulationStep();
  sim_initialized_ = true;

  if (!headless_) {
    renderer_.reset(new MujocoRenderer(this));
    renderer_->launchRenderThread();
  }
}

void MujocoSimInterface::start() {
  if (!sim_initialized_) initSim();
  if (simulate_thread_.joinable()) {
    std::cerr << "WARNING: Tried to start simulation thread, but it is already running." << std::endl;
    return;
  }
  simulate_thread_ = std::jthread([this](std::stop_token st) { this->simulationLoop(st); });
}

void MujocoSimInterface::stop() {
  if (simulate_thread_.joinable()) {
    simulate_thread_.request_stop();
    if (simulate_thread_.get_id() != std::this_thread::get_id()) {
      simulate_thread_.join();
    }
  }
}

}  // namespace motorium::mujoco