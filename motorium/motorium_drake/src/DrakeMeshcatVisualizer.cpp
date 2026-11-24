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

#include "motorium_drake/DrakeMeshcatVisualizer.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <vector>

#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_params.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>

#include <motorium_drake/DrakeDriver.h>
#include <motorium_model/RobotState.h>

#include "DrakeModelLoading.h"

namespace motorium::drake_sim {

namespace dgeo = ::drake::geometry;
namespace dmb = ::drake::multibody;
namespace dsys = ::drake::systems;

DrakeMeshcatVisualizer::DrakeMeshcatVisualizer(const DrakeDriver& driver,
                                               const model::RobotDescription& robot_description,
                                               const DrakeSimConfig& config,
                                               const std::string& model_path)
    : driver_(driver), robot_description_(robot_description), config_(config), model_path_(model_path) {
  std::promise<std::string> ready;
  std::future<std::string> url = ready.get_future();
  render_thread_ = std::jthread([this, &ready](std::stop_token stop_token) { renderLoop(stop_token, ready); });
  url_ = url.get();
}

void DrakeMeshcatVisualizer::renderLoop(std::stop_token stop_token, std::promise<std::string>& ready) {
  std::shared_ptr<dgeo::Meshcat> meshcat;
  std::unique_ptr<dsys::Diagram<double>> diagram;
  std::unique_ptr<dsys::Context<double>> context;
  const dmb::MultibodyPlant<double>* plant = nullptr;
  dsys::Context<double>* plant_context = nullptr;
  const dmb::RigidBody<double>* root_body = nullptr;
  bool is_floating_base = false;
  std::vector<int> joint_q_index;  // per RobotDescription joint: index of its position in the plant's q

  try {
    dsys::DiagramBuilder<double> builder;
    auto [new_plant, scene_graph] = dmb::AddMultibodyPlantSceneGraph(&builder, 0.0);
    const dmb::ModelInstanceIndex model = internal::addRobotModel(new_plant, config_.packagePaths, model_path_);
    internal::addGround(new_plant, config_.ground, /*with_collision=*/false);
    new_plant.Finalize();
    root_body = &internal::findRootBody(new_plant, model, is_floating_base);
    for (const std::string& joint_name : robot_description_.getJointNames()) {
      joint_q_index.push_back(new_plant.GetJointByName(joint_name, model).position_start());
    }

    dgeo::MeshcatParams meshcat_params;
    meshcat_params.host = config_.meshcatHost;
    meshcat_params.port = config_.meshcatPort;
    meshcat = std::make_shared<dgeo::Meshcat>(meshcat_params);
    dgeo::MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat);

    plant = &new_plant;
    diagram = builder.Build();
    context = diagram->CreateDefaultContext();
    plant_context = &plant->GetMyMutableContextFromRoot(context.get());
  } catch (...) {
    ready.set_exception(std::current_exception());
    return;
  }
  ready.set_value(meshcat->web_url());  // the constructor's `ready` is gone after this line

  model::RobotState state(robot_description_, std::max<size_t>(1, config_.contactBodyNames.size()));
  Eigen::VectorXd q(plant->num_positions());
  const auto period =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / config_.visualizationRateHz));
  auto next_frame = std::chrono::steady_clock::now();

  while (!stop_token.stop_requested()) {
    driver_.copyStateSnapshot(state);
    if (is_floating_base) {
      const quaternion_t orientation = state.getRootRotationLocalToWorldFrame().normalized();
      plant->SetFreeBodyPose(plant_context, *root_body,
                             ::drake::math::RigidTransformd(::drake::math::RotationMatrixd(orientation.toRotationMatrix()),
                                                            state.getRootPositionInWorldFrame()));
    }
    q = plant->GetPositions(*plant_context);
    for (size_t i = 0; i < joint_q_index.size(); ++i) {
      q(joint_q_index[i]) = state.getJointPosition(i);
    }
    plant->SetPositions(plant_context, q);
    diagram->ForcedPublish(*context);

    next_frame = std::max(next_frame + period, std::chrono::steady_clock::now());
    std::this_thread::sleep_until(next_frame);
  }
}

}  // namespace motorium::drake_sim
