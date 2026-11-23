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

// Model loading helpers shared by DrakeSimulation and DrakeMeshcatVisualizer (private to this package).

#include <algorithm>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include <drake/geometry/proximity_properties.h>
#include <drake/geometry/shape_specification.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>

#include <motorium_core/Check.h>
#include <motorium_drake/DrakeSimConfig.h>

namespace motorium::drake_sim::internal {

// Loads the robot model (URDF or SDF) into the plant and returns its model instance.
inline ::drake::multibody::ModelInstanceIndex addRobotModel(::drake::multibody::MultibodyPlant<double>& plant,
                                                            const std::vector<std::pair<std::string, std::string>>& package_paths,
                                                            const std::string& model_path) {
  ::drake::multibody::Parser parser(&plant);
  for (const auto& [name, path] : package_paths) {
    parser.package_map().Add(name, path);
  }
  const auto models = parser.AddModels(model_path);
  MT_CHECK(models.size() == 1) << "[DrakeDriver] Expected exactly one model in " << model_path << ", found " << models.size();
  return models.front();
}

// The ground box of GroundConfig (no-op if disabled). Collision geometry only for the physics plant.
inline void addGround(::drake::multibody::MultibodyPlant<double>& plant, const GroundConfig& ground, bool with_collision) {
  if (!ground.enabled) return;

  const ::drake::math::RigidTransformd X_WG(Eigen::Vector3d(0.0, 0.0, -0.1));
  const ::drake::geometry::Box box(20.0, 20.0, 0.2);
  if (with_collision) {
    ::drake::geometry::ProximityProperties properties;
    ::drake::geometry::AddContactMaterial(ground.huntCrossleyDissipation, ground.pointStiffness,
                                          ::drake::multibody::CoulombFriction<double>(ground.staticFriction, ground.dynamicFriction),
                                          &properties);
    plant.RegisterCollisionGeometry(plant.world_body(), X_WG, box, "ground_collision", properties);
  }
  plant.RegisterVisualGeometry(plant.world_body(), X_WG, box, "ground_visual", Eigen::Vector4d(0.5, 0.5, 0.5, 1.0));
}

// Bodies are reached through the model's joints rather than MultibodyPlant::get_body(BodyIndex).
// Drake instantiates its body collection with the alias template `Link = RigidBody`, which GCC and
// clang mangle differently, so a clang-compiled get_body() call does not link against a GCC-built Drake.

// All bodies of the model instance that are connected by a joint.
inline std::vector<const ::drake::multibody::RigidBody<double>*> collectModelBodies(const ::drake::multibody::MultibodyPlant<double>& plant,
                                                                                    ::drake::multibody::ModelInstanceIndex model) {
  std::vector<const ::drake::multibody::RigidBody<double>*> bodies;
  const auto add = [&](const ::drake::multibody::RigidBody<double>& body) {
    if (body.model_instance() == model && std::find(bodies.begin(), bodies.end(), &body) == bodies.end()) {
      bodies.push_back(&body);
    }
  };
  for (const ::drake::multibody::JointIndex& joint_index : plant.GetJointIndices(model)) {
    const ::drake::multibody::Joint<double>& joint = plant.get_joint(joint_index);
    add(joint.parent_body());
    add(joint.child_body());
  }
  return bodies;
}

// The body reported as the RobotState root: the floating base if the model has one, otherwise the body
// attached to the world. Call after Finalize().
inline const ::drake::multibody::RigidBody<double>& findRootBody(const ::drake::multibody::MultibodyPlant<double>& plant,
                                                                 ::drake::multibody::ModelInstanceIndex model,
                                                                 bool& is_floating_base) {
  is_floating_base = plant.HasUniqueFloatingBaseBody(model);
  if (is_floating_base) {
    return plant.GetUniqueFloatingBaseBodyOrThrow(model);
  }
  for (const ::drake::multibody::JointIndex& joint_index : plant.GetJointIndices(model)) {
    const ::drake::multibody::Joint<double>& joint = plant.get_joint(joint_index);
    if (joint.parent_body().index() == ::drake::multibody::world_index()) {
      return joint.child_body();
    }
  }
  MT_CHECK(false) << "[DrakeDriver] The model has neither a floating base nor a body attached to the world";
  std::abort();  // unreachable
}

}  // namespace motorium::drake_sim::internal
