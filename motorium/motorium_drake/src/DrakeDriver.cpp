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

#include "motorium_drake/DrakeDriver.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <iostream>
#include <string>

#include <motorium_core/Check.h>
#include <motorium_drake/DrakeMeshcatVisualizer.h>

#include "DrakeSimulation.h"

namespace motorium::drake_sim {

namespace {

// With config.verbose, the physics thread prints timing metrics this often.
constexpr std::chrono::seconds kReportPeriod{5};

// Copies the state from src to dst without allocating. Contact flags only when the driver computes
// them, so a caller's flags are otherwise left untouched.
void copyRobotState(const model::RobotState& src, model::RobotState& dst, size_t num_joints, size_t num_contact_flags) {
  dst.setRootPositionInWorldFrame(src.getRootPositionInWorldFrame());
  dst.setRootRotationLocalToWorldFrame(src.getRootRotationLocalToWorldFrame());
  dst.setRootLinearVelocityInLocalFrame(src.getRootLinearVelocityInLocalFrame());
  dst.setRootAngularVelocityInLocalFrame(src.getRootAngularVelocityInLocalFrame());
  for (size_t i = 0; i < num_joints; ++i) {
    dst.setJointState(i, src.getJointState(i));
  }
  dst.setTime(src.getTime());
  for (size_t i = 0; i < num_contact_flags; ++i) {
    dst.setContactFlag(i, src.getContactFlag(i));
  }
}

}  // namespace

DrakeDriver::DrakeDriver(hal::HalKey key, const model::RobotDescription& robot_description, const DrakeSimConfig& config)
    : DriverBase(key, robot_description, "drake_sim"),
      config_(config),
      robot_description_(robot_description),
      initial_state_(robot_description),
      action_(robot_description),
      snapshot_(robot_description, std::max<size_t>(1, config.contactBodyNames.size())) {
  config_.validate();

  if (config_.initStatePtr != nullptr) {
    initial_state_ = *config_.initStatePtr;
  } else {
    initial_state_.setConfigurationToZero();
    initial_state_.setRootPositionInWorldFrame(vector3_t(0.0, 0.0, 1.0));
  }
  if (config_.initialActionPtr != nullptr) {
    MT_CHECK(config_.initialActionPtr->size() == robot_description.getNumJoints())
        << "[DrakeDriver] initialActionPtr has " << config_.initialActionPtr->size() << " joints, the RobotDescription has "
        << robot_description.getNumJoints();
  }

  const std::string& model_path = config_.modelPath.empty() ? robot_description.getModelPath() : config_.modelPath;
  MT_CHECK(!model_path.empty()) << "[DrakeDriver] No model to load: DrakeSimConfig::modelPath is empty and the RobotDescription "
                                   "was not created from a model file.";
  {
    absl::MutexLock lock(&sim_mutex_);
    simulation_ = std::make_unique<DrakeSimulation>(robot_description, config_, model_path);
    restoreInitialState();
  }

  if (!config_.headless) {
    visualizer_ = std::make_unique<DrakeMeshcatVisualizer>(*this, robot_description, config_, model_path);
    std::cerr << "[DrakeDriver] Meshcat: " << visualizer_->getUrl() << std::endl;
  }

  markReady();
}

DrakeDriver::~DrakeDriver() = default;  // members stop the visualizer and physics threads first

void DrakeDriver::copyStateSnapshot(model::RobotState& state) const {
  absl::MutexLock lock(&snapshot_mutex_);
  copyRobotState(snapshot_, state, robot_description_.getNumJoints(), config_.contactBodyNames.size());
}

DrakeMetrics DrakeDriver::getMetrics() const {
  absl::MutexLock lock(&snapshot_mutex_);
  return metrics_;
}

std::string DrakeDriver::getMeshcatUrl() const {
  return visualizer_ != nullptr ? visualizer_->getUrl() : std::string();
}

bool DrakeDriver::advance(size_t num_steps) {
  MT_CHECK(config_.manualStepping) << "[DrakeDriver] advance() requires DrakeSimConfig::manualStepping";
  for (size_t i = 0; i < num_steps; ++i) {
    if (getState() != hal::DriverState::RUNNING) return false;
    absl::MutexLock lock(&sim_mutex_);
    if (!step()) return false;
  }
  return true;
}

// DriverBase only calls this from READY, so no physics thread is running.
void DrakeDriver::startImpl() {
  if (config_.manualStepping) return;
  physics_thread_ = std::jthread([this](std::stop_token stop_token) { physicsLoop(stop_token); });
}

void DrakeDriver::stopImpl() {
  if (physics_thread_.joinable()) {
    physics_thread_.request_stop();
    physics_thread_.join();
  }
}

void DrakeDriver::resetImpl() {
  absl::MutexLock lock(&sim_mutex_);
  restoreInitialState();
}

void DrakeDriver::updateStateImpl(model::RobotState& robot_state) {
  copyStateSnapshot(robot_state);
}

void DrakeDriver::setActionImpl(const model::RobotJointFeedbackAction& action) {
  absl::MutexLock lock(&action_mutex_);
  action_ = action;
  action_received_ = true;
}

bool DrakeDriver::step() {
  const auto start = std::chrono::steady_clock::now();
  try {
    {
      // Torques are computed under the action lock (microseconds); the step itself runs without it,
      // so setAction() never waits for a physics step.
      absl::MutexLock lock(&action_mutex_);
      const bool hold_initial_action = !action_received_ && config_.initialActionPtr != nullptr;
      simulation_->applyAction(hold_initial_action ? *config_.initialActionPtr : action_);
    }
    simulation_->advance();
  } catch (const std::exception& error) {
    reportFault(std::string("Drake simulation step failed: ") + error.what());
    return false;
  }
  const double step_time_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();

  {
    absl::MutexLock lock(&snapshot_mutex_);
    simulation_->readState(snapshot_);
    metrics_.stepCount++;
    metrics_.meanStepTimeUs += (step_time_us - metrics_.meanStepTimeUs) / static_cast<double>(metrics_.stepCount);
    metrics_.maxStepTimeUs = std::max(metrics_.maxStepTimeUs, step_time_us);
  }

  if (config_.faultBelowRootHeight.has_value() && simulation_->isFloatingBase() &&
      simulation_->rootHeight() < *config_.faultBelowRootHeight) {
    reportFault("floating base height " + std::to_string(simulation_->rootHeight()) + " m is below faultBelowRootHeight " +
                std::to_string(*config_.faultBelowRootHeight) + " m");
    return false;
  }
  return true;
}

void DrakeDriver::restoreInitialState() {
  simulation_->setState(initial_state_);
  {
    absl::MutexLock lock(&action_mutex_);
    action_received_ = false;
  }
  absl::MutexLock lock(&snapshot_mutex_);
  simulation_->readState(snapshot_);
}

void DrakeDriver::physicsLoop(std::stop_token stop_token) {
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(config_.dt));
  auto next_step = std::chrono::steady_clock::now();
  auto next_report = next_step + kReportPeriod;
  uint64_t steps_since_report = 0;

  while (!stop_token.stop_requested()) {
    {
      absl::MutexLock lock(&sim_mutex_);
      if (!step()) return;  // faulted: no more steps until reset and restart
    }

    ++steps_since_report;
    if (config_.verbose && std::chrono::steady_clock::now() >= next_report) {
      // Real-time factor: simulated time per wall-clock time over the last report period.
      const double real_time_factor =
          static_cast<double>(steps_since_report) * config_.dt / std::chrono::duration<double>(kReportPeriod).count();
      const DrakeMetrics metrics = getMetrics();
      std::cerr << "[DrakeDriver] real-time factor " << real_time_factor << ", step mean " << metrics.meanStepTimeUs << " us, max "
                << metrics.maxStepTimeUs << " us, overruns " << metrics.overrunCount << std::endl;
      steps_since_report = 0;
      next_report += kReportPeriod;
    }

    next_step += period;
    const auto now = std::chrono::steady_clock::now();
    if (now > next_step + period) {
      // More than one step behind: resynchronize instead of catching up in a burst.
      next_step = now;
      absl::MutexLock lock(&snapshot_mutex_);
      metrics_.overrunCount++;
    } else {
      std::this_thread::sleep_until(next_step);
    }
  }
}

}  // namespace motorium::drake_sim
