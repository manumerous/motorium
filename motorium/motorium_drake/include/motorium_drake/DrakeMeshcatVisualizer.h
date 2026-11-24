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

#include <future>
#include <stop_token>
#include <string>
#include <thread>

#include <motorium_drake/DrakeSimConfig.h>
#include <motorium_model/RobotDescription.h>

namespace motorium::drake_sim {

class DrakeDriver;

// Shows a DrakeDriver's robot in Meshcat (a browser-based 3D viewer). Created by DrakeDriver when
// DrakeSimConfig::headless is false.
//
// Runs on its own thread with its own visual-only plant, which it poses from the driver's state
// snapshot at visualizationRateHz, so it never touches the physics. Drake requires every Meshcat call
// on the thread that created the Meshcat instance, so the scene and server live entirely on that thread.
class DrakeMeshcatVisualizer {
 public:
  // Blocks until the Meshcat server is up; rethrows any error from setting up the scene.
  DrakeMeshcatVisualizer(const DrakeDriver& driver,
                         const model::RobotDescription& robot_description,
                         const DrakeSimConfig& config,
                         const std::string& model_path);

  DrakeMeshcatVisualizer(const DrakeMeshcatVisualizer&) = delete;
  DrakeMeshcatVisualizer& operator=(const DrakeMeshcatVisualizer&) = delete;

  const std::string& getUrl() const { return url_; }

 private:
  // Builds the scene and starts Meshcat, fulfils `ready` with the URL (or the setup error), then
  // publishes the driver's state until stopped.
  void renderLoop(std::stop_token stop_token, std::promise<std::string>& ready);

  const DrakeDriver& driver_;
  const model::RobotDescription& robot_description_;
  const DrakeSimConfig config_;
  const std::string model_path_;
  std::string url_;

  std::jthread render_thread_;  // declared last: stopped and joined before the other members go away
};

}  // namespace motorium::drake_sim
