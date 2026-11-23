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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stop_token>
#include <string>
#include <thread>

#include <absl/base/thread_annotations.h>
#include <absl/synchronization/mutex.h>

#include <motorium_drake/DrakeSimConfig.h>
#include <motorium_hal/DriverBase.h>
#include <motorium_model/RobotDescription.h>
#include <motorium_model/RobotJointFeedbackAction.h>
#include <motorium_model/RobotState.h>

namespace motorium::drake_sim {

class DrakeSimulation;
class DrakeMeshcatVisualizer;

// Timing of the physics steps, to check whether the simulation keeps up with real time.
struct DrakeMetrics {
  uint64_t stepCount{0};
  double meanStepTimeUs{0.0};  // compute time of one step [us]
  double maxStepTimeUs{0.0};
  uint64_t overrunCount{0};  // times the physics thread fell more than one step behind real time
};

// Simulates the robot with Drake behind the motorium HAL. Created through RobotHAL::addDriver().
//
// Every physics step applies each joint action as kp * (q_des - q) + kd * (v_des - v) + feed_forward_effort,
// evaluated at the current state and clamped to the joint's torque bounds.
//
// Threads:
//  - The physics thread steps the simulation every dt in real time (unless manualStepping).
//  - updateState() / setAction() only copy the latest state / action under short locks; they never
//    wait for a physics step.
//  - An optional Meshcat thread shows the robot (DrakeMeshcatVisualizer).
//
// Faults: a Drake error, a non-finite state, or the floating base dropping below faultBelowRootHeight
// put the driver into FAULT and stop the physics. RobotHAL then safety-stops; resetDrivers() restores
// the initial state.
class DrakeDriver final : public hal::DriverBase {
 public:
  DrakeDriver(hal::HalKey key, const model::RobotDescription& robot_description, const DrakeSimConfig& config);
  ~DrakeDriver() override;

  DrakeDriver(const DrakeDriver&) = delete;
  DrakeDriver& operator=(const DrakeDriver&) = delete;

  // Copies the latest simulated state, in any driver state. Contact flags are written only when
  // contactBodyNames is set. Thread-safe.
  void copyStateSnapshot(model::RobotState& state) const;

  DrakeMetrics getMetrics() const;

  // Meshcat URL, empty when headless.
  std::string getMeshcatUrl() const;

  const DrakeSimConfig& getConfig() const { return config_; }

  // Manual stepping only: advances the simulation by num_steps steps on the calling thread. Returns
  // false if the driver is not RUNNING or faults while stepping.
  bool advance(size_t num_steps = 1);

 private:
  // DriverBase lifecycle hooks; DriverBase checks and performs the state transitions around them.
  void startImpl() override;
  void stopImpl() override;
  void resetImpl() override;
  void updateStateImpl(model::RobotState& robot_state) override;
  void setActionImpl(const model::RobotJointFeedbackAction& action) override;

  // One physics step plus state snapshot. Returns false if it faulted.
  bool step() ABSL_EXCLUSIVE_LOCKS_REQUIRED(sim_mutex_);
  // Restores the initial state and forgets the last received action.
  void restoreInitialState() ABSL_EXCLUSIVE_LOCKS_REQUIRED(sim_mutex_);
  void physicsLoop(std::stop_token stop_token);

  const DrakeSimConfig config_;
  const model::RobotDescription& robot_description_;
  model::RobotState initial_state_;

  absl::Mutex sim_mutex_;
  std::unique_ptr<DrakeSimulation> simulation_ ABSL_PT_GUARDED_BY(sim_mutex_);

  mutable absl::Mutex action_mutex_;
  model::RobotJointFeedbackAction action_ ABSL_GUARDED_BY(action_mutex_);
  bool action_received_ ABSL_GUARDED_BY(action_mutex_){false};

  mutable absl::Mutex snapshot_mutex_;
  model::RobotState snapshot_ ABSL_GUARDED_BY(snapshot_mutex_);
  DrakeMetrics metrics_ ABSL_GUARDED_BY(snapshot_mutex_);

  // Declared last: destroyed first, so neither thread outlives the members it uses.
  std::jthread physics_thread_;
  std::unique_ptr<DrakeMeshcatVisualizer> visualizer_;
};

}  // namespace motorium::drake_sim
