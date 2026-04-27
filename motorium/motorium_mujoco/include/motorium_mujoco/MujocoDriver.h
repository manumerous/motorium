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
  double defaultJointDamping{1.0};
};

class MujocoDriver final : public hal::DriverBase {
 public:
  MujocoDriver(hal::HalKey key, const model::RobotDescription& robot_description, const MujocoSimConfig& config);

  /** Destructor */
  ~MujocoDriver();

  void startImpl() override;

  void stopImpl() override;

  void simulationStep();

  // Todo Manu also reset environment
  void resetImpl() override;

  // Allows the renderer to make a thread safe copy of the state at it's own
  // frequency.
  void copyMjState(MjState& state) const;

  const mjModel* getModel() const { return mj_model_; }

  const MujocoSimConfig& getConfig() const { return config_; }

 private:
  void initSim();

  void setupJointIndexMaps(const model::RobotDescription& robot_description);

  void setSimState(const model::RobotState& robot_state);

  void simulationLoop(std::stop_token st);

  void updateImpl(const model::RobotJointFeedbackAction& action, model::RobotState& robot_state) override;

  void printModelInfo();

  void updateMetrics();

  MujocoSimConfig config_;

  bool is_floating_base_{false};
  size_t nq_base_offset_{0};
  size_t nv_base_offset_{0};

  std::vector<mjtNum> qpos_init_;  // position                                         (nq x 1)
  std::vector<mjtNum> qvel_init_;
  absl::Mutex action_mutex_;
  model::RobotJointFeedbackAction action_internal_ ABSL_GUARDED_BY(action_mutex_);

  size_t time_step_micro_;
  size_t num_active_joints_;
  size_t num_actuators_;

  // Keeping tack of active joints/actuators to allow for the use of e.g passive or mimic joints
  std::vector<joint_index_t> active_robot_joint_indices_;
  std::vector<joint_index_t> active_robot_actuator_indices_;

  // TODO: Move to non-blocking buffer in the future.
  mutable absl::Mutex mj_mutex_;  // Used to access data accross simulation and render threads.

  mjModel* mj_model_ = NULL;
  mjData* mj_data_ ABSL_GUARDED_BY(mj_mutex_) = NULL;

  bool sim_initialized_{false};
  bool reset_requested_{false};

  std::jthread simulate_thread_;
  std::unique_ptr<MujocoRenderer> renderer_;

  FPSTracker simFps_{"mujoco_sim"};
  std::chrono::high_resolution_clock::time_point last_realtime_;
  Metrics metrics_{};

  double drift_mean_sq_{0.0};
};

}  // namespace motorium::mujoco
