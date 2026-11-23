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

#include "motorium_drake/DrakeSimConfig.h"

#include <cmath>

#include <motorium_core/Check.h>

namespace motorium::drake_sim {

void DrakeSimConfig::validate() const {
  MT_CHECK(std::isfinite(dt) && dt > 0.0) << "[DrakeSimConfig] dt must be positive and finite, got " << dt;
  MT_CHECK(defaultJointDamping >= 0.0) << "[DrakeSimConfig] defaultJointDamping must be non-negative, got " << defaultJointDamping;

  if (ground.enabled) {
    MT_CHECK(ground.dynamicFriction >= 0.0 && ground.staticFriction >= ground.dynamicFriction)
        << "[DrakeSimConfig] ground friction must satisfy static friction >= dynamic friction >= 0, got static " << ground.staticFriction
        << " and dynamic " << ground.dynamicFriction;
    MT_CHECK(ground.huntCrossleyDissipation >= 0.0) << "[DrakeSimConfig] ground dissipation must be non-negative";
    MT_CHECK(ground.pointStiffness > 0.0) << "[DrakeSimConfig] ground point stiffness must be positive";
  }

  if (faultBelowRootHeight.has_value()) {
    MT_CHECK(std::isfinite(*faultBelowRootHeight)) << "[DrakeSimConfig] faultBelowRootHeight must be finite";
  }

  if (!headless) {
    MT_CHECK(visualizationRateHz > 0.0) << "[DrakeSimConfig] visualizationRateHz must be positive, got " << visualizationRateHz;
    if (meshcatPort.has_value()) {
      MT_CHECK(*meshcatPort == 0 || (*meshcatPort >= 1024 && *meshcatPort <= 65535))
          << "[DrakeSimConfig] meshcatPort must be 0 or in [1024, 65535], got " << *meshcatPort;
    }
  }
}

}  // namespace motorium::drake_sim
