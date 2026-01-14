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

#pragma once

#include <iostream>
#include <string>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <atomic>
#include <chrono>
#include <ctime>
#include <mutex>
#include <thread>

#include <Eigen/Dense>

#include <motorium_model/RobotState.h>
#include "motorium_core/FPSTracker.h"
#include "motorium_core/Types.h"
#include "motorium_hal/DriverBase.h"
#include "motorium_mujoco/MujocoRenderer.h"
#include "motorium_mujoco/MujocoUtils.h"

namespace motorium::mujoco {

struct MujocoSimConfig {
  std::string scenePath;
  std::shared_ptr<model::RobotState> initStatePtr_;
  double dt{0.0005};
  double renderFrequencyHz{60.0};
  bool headless{false};
  bool verbose{false};
};

class MujocoSimInterface : public hal::DriverBase {
 public:
  MujocoSimInterface(const MujocoSimConfig& config, const model::RobotDescription& robot_description);

  /** Destructor */
  ~MujocoSimInterface();

  void initSim();

  void start() override;

  void stop() override;

  void updateRobotState(model::RobotState& robot_state) override;

  void setJointAction(const model::RobotJointAction& action) override;

  void simulationStep();

  // Todo Manu also reset environment
  void reset() override;

  // Allows the renderer to make a thread safe copy of the state at it's own
  // frequency.
  void copyMjState(MjState& state) const;

  const mjModel* getModel() const { return mj_model_; }

  const MujocoSimConfig& getConfig() const { return config_; }

 private:
  void setupJointIndexMaps(const model::RobotDescription& robot_description);

  void setSimState(const model::RobotState& robot_state);

  void updateThreadSafeRobotState();

  void simulationLoop(std::stop_token st);

  void printModelInfo();

  void updateMetrics();

  MujocoSimConfig config_;

  mjtNum* qpos_init_;  // position                                         (nq x 1)
  mjtNum* qvel_init_;
  model::RobotJointAction action_internal_;
  mutable std::mutex action_mutex_;

  size_t time_step_micro_;
  size_t num_active_joints_;
  size_t num_actuators_;
  std::vector<std::string> active_joint_names_;
  std::vector<std::string> active_actuator_names_;
  std::vector<joint_index_t> active_robot_joint_indices_;
  std::vector<joint_index_t> active_robot_actuator_indices_;

  mjModel* mj_model_ = NULL;
  mjData* mj_data_ = NULL;
  mjContact* mj_contact_ = NULL;
  // mjfSensor mujocoSenor_;

  bool sim_initialized_;
  const bool headless_;
  const bool verbose_;

  mutable std::mutex mj_mutex_;  // Used to access mujoco model and data
                                    // accross simulation and render threads.
  std::jthread simulate_thread_;
  std::unique_ptr<MujocoRenderer> renderer_;

  FPSTracker simFps_{"mujoco_sim"};
  std::chrono::high_resolution_clock::time_point last_realtime_;
  Metrics metrics_{};

  size_t right_foot_sensor_addr_;
  size_t left_foot_sensor_addr_;

  size_t right_foot_touch_sensor_addr_;
  size_t left_foot_touch_sensor_addr_;
};

}  // namespace motorium::mujoco
