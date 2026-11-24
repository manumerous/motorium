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

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <motorium_model/RobotJointFeedbackAction.h>
#include <motorium_model/RobotState.h>

namespace motorium::drake_sim {

// Flat ground: a 20 x 20 x 0.2 m box whose top face is at z = 0.
struct GroundConfig {
  bool enabled{true};
  double staticFriction{2.0};
  double dynamicFriction{1.5};
  double huntCrossleyDissipation{2.0};
  double pointStiffness{5.0e4};  // [N/m]
};

// Configuration of a DrakeDriver. The defaults are the values the Unitree G1 walked with in the
// legacy wb_humanoid_mpc Drake simulation.
struct DrakeSimConfig {
  // URDF or SDF file to simulate. Empty -> the model path of the RobotDescription. The model must
  // contain every joint of the RobotDescription as a one-degree-of-freedom (revolute or prismatic) joint.
  std::string modelPath;

  // Resolves package:// URIs (e.g. meshes) in the model: {package name, package directory}.
  std::vector<std::pair<std::string, std::string>> packagePaths;

  // State at construction and after every reset. nullptr -> all joints at zero, floating base at z = 1 m.
  std::shared_ptr<const model::RobotState> initStatePtr;

  // Action applied from construction, and again after every reset, until the first setAction()
  // arrives. Lets the robot hold its pose while the controller starts up. nullptr -> zero torque.
  std::shared_ptr<const model::RobotJointFeedbackAction> initialActionPtr;

  double dt{0.0005};                 // physics time step [s]
  double defaultJointDamping{10.0};  // viscous damping of every joint [Nms/rad]
  GroundConfig ground;

  // Bodies whose ground contact sets RobotState contact flag i (e.g. the feet). Empty -> the driver
  // never writes contact flags. The RobotState passed to updateState() needs at least this many flags.
  std::vector<std::string> contactBodyNames;

  // Reports a FAULT when the floating base drops below this height [m]. nullopt -> disabled.
  std::optional<double> faultBelowRootHeight;

  // true: no physics thread; the simulation advances only through DrakeDriver::advance(). Deterministic
  // and faster than real time, e.g. for tests.
  bool manualStepping{false};

  // Meshcat visualization. headless = true -> no visualizer.
  bool headless{true};
  double visualizationRateHz{30.0};
  std::string meshcatHost{"localhost"};  // "*" listens on all interfaces
  std::optional<int> meshcatPort;        // nullopt -> first free port from 7000; 0 -> any free port

  // Prints model loading details and, every 5 s, the real-time factor and step-time metrics.
  bool verbose{false};

  // Aborts with a descriptive message on invalid values.
  void validate() const;
};

}  // namespace motorium::drake_sim
