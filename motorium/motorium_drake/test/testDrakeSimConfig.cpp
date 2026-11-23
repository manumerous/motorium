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

#include <gtest/gtest.h>

#include <motorium_drake/DrakeSimConfig.h>

using motorium::drake_sim::DrakeSimConfig;

TEST(DrakeSimConfigTest, DefaultsAreValid) {
  DrakeSimConfig config;
  config.validate();
  config.headless = false;
  config.validate();
  SUCCEED();
}

TEST(DrakeSimConfigTest, DefaultsMatchLegacyWalkingSetup) {
  const DrakeSimConfig config;
  EXPECT_DOUBLE_EQ(config.dt, 0.0005);
  EXPECT_DOUBLE_EQ(config.defaultJointDamping, 10.0);
  EXPECT_DOUBLE_EQ(config.ground.staticFriction, 2.0);
  EXPECT_DOUBLE_EQ(config.ground.dynamicFriction, 1.5);
  EXPECT_DOUBLE_EQ(config.ground.huntCrossleyDissipation, 2.0);
  EXPECT_DOUBLE_EQ(config.ground.pointStiffness, 5.0e4);
}

TEST(DrakeSimConfigDeathTest, RejectsNonPositiveDt) {
  DrakeSimConfig config;
  config.dt = 0.0;
  EXPECT_DEATH(config.validate(), "dt must be positive");
}

TEST(DrakeSimConfigDeathTest, RejectsNegativeDamping) {
  DrakeSimConfig config;
  config.defaultJointDamping = -1.0;
  EXPECT_DEATH(config.validate(), "defaultJointDamping");
}

TEST(DrakeSimConfigDeathTest, RejectsStaticFrictionBelowDynamic) {
  DrakeSimConfig config;
  config.ground.staticFriction = 0.5;
  config.ground.dynamicFriction = 1.0;
  EXPECT_DEATH(config.validate(), "static friction >= dynamic friction");
}

TEST(DrakeSimConfigTest, IgnoresGroundValuesWhenGroundDisabled) {
  DrakeSimConfig config;
  config.ground.enabled = false;
  config.ground.staticFriction = -1.0;
  config.validate();
  SUCCEED();
}

TEST(DrakeSimConfigDeathTest, RejectsNonFiniteFaultHeight) {
  DrakeSimConfig config;
  config.faultBelowRootHeight = std::numeric_limits<double>::infinity();
  EXPECT_DEATH(config.validate(), "faultBelowRootHeight");
}

TEST(DrakeSimConfigDeathTest, RejectsBadVisualizationSettingsOnlyWhenNotHeadless) {
  DrakeSimConfig config;
  config.visualizationRateHz = 0.0;
  config.meshcatPort = 80;
  config.validate();  // headless: ignored

  config.headless = false;
  EXPECT_DEATH(config.validate(), "visualizationRateHz");
  config.visualizationRateHz = 30.0;
  EXPECT_DEATH(config.validate(), "meshcatPort");
}
