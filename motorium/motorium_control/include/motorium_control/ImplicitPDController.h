#pragma once

#include <motorium_control/ControllerBase.h>
#include <motorium_core/Check.h>
#include <motorium_core/Types.h>

#include <string>
#include <vector>

namespace motorium::control {

struct ImplicitPDControllerConfig {
  std::vector<std::string> joint_names;
  vector_t kp;
  vector_t kd;
};

class ImplicitPDController : public ControllerBase {
 public:
  ImplicitPDController(const model::RobotDescription& robot_description, const ImplicitPDControllerConfig& config);
  ~ImplicitPDController() override = default;

  bool validateConfig() const;

  void computeJointControlAction(scalar_t time, const model::RobotState& robot_state, model::RobotJointAction& robot_joint_action) override;

 private:
  ImplicitPDControllerConfig config_;
  std::vector<joint_index_t> joint_indices_;
};

}  // namespace motorium::control
