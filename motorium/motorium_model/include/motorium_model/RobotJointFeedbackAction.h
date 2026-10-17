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

#include <motorium_core/Bounds.h>
#include <motorium_core/Types.h>
#include <motorium_model/JointIDMap.h>
#include <motorium_model/RobotDescription.h>

#include <cstddef>
#include <limits>

namespace motorium::model {

// Added to an accumulated gain before dividing by it, so that a joint with no accumulated
// stiffness yields 0 instead of 0 / 0 = NaN. See toJointFeedbackAction below for why that
// NaN is fatal rather than cosmetic.
//
// This is a guard, not a tolerance: it never trades accuracy for safety. Doubles carry a fixed
// relative precision, so adding a value far below a gain's ULP rounds straight back to the gain
// — 2.2e-308 only perturbs magnitudes near 1e-292, which no physical stiffness or damping
// approaches. Every realistic gain divides bit-identically to the unguarded form, while a gain
// of exactly 0 divides an exactly-zero numerator (every kp * q_des term was 0) and gives 0.
//
// The smallest *normal* double is deliberate. Subnormals go below it, but flush-to-zero and
// denormals-are-zero modes discard those, which would silently reinstate the 0 / 0 under
// optimization.
constexpr scalar_t kGainFloor = std::numeric_limits<scalar_t>::min();

// An output scaler that can be used to scale the output of a controller from 0% to 100%. This can
// be used to smoothly ramp in controllers or scale their output down for testing.
inline const Bounds kOutputScalerBounds{0.0, 1.0};

class OutputScaler {
 public:
  explicit OutputScaler(scalar_t value) : value_(kOutputScalerBounds.clamp(value)) {}
  scalar_t getValue() const { return value_; }
  void setValue(scalar_t value) { value_ = kOutputScalerBounds.clamp(value); }

 private:
  scalar_t value_;
};

inline const OutputScaler kFullOutputScaler{1.0};

template <typename SCALAR_T>
struct JointFeedbackActionTpl {
  SCALAR_T q_des = 0;
  SCALAR_T v_des = 0;
  SCALAR_T kp = 0;
  SCALAR_T kd = 0;
  SCALAR_T feed_forward_effort = 0;

  SCALAR_T getTotalFeedbackTorque(SCALAR_T q, SCALAR_T v) const { return (kp * (q_des - q) + kd * (v_des - v) + feed_forward_effort); }

  // Returns a scaled copy so the original is not accidentally modified. Setpoints are kept, only
  // gains and feed forward effort are scaled.
  JointFeedbackActionTpl scaled(SCALAR_T factor) const {
    return {.q_des = q_des, .v_des = v_des, .kp = kp * factor, .kd = kd * factor, .feed_forward_effort = feed_forward_effort * factor};
  }
};

// A linear transformation of a feedback action that makes them stackable and allows aggregating the
// effect of different feedback controllers without ever needing to evaluate the feedback terms,
// which can then be computed at a higher feedback rate (e.g. on the motor driver).
template <typename SCALAR_T>
struct JointAccumulactionTpl {
  SCALAR_T q_des_feed_forward = 0;
  SCALAR_T v_des_feed_forward = 0;
  SCALAR_T kp_feedback = 0;
  SCALAR_T kd_feedback = 0;
  SCALAR_T feed_forward_effort = 0;

  // The OutputScaler is an explicit argument so that the scaling ability is not forgotten, e.g. in
  // the controllers.
  void addFeedbackAction(const JointFeedbackActionTpl<SCALAR_T>& action, OutputScaler output_scaler) {
    const JointFeedbackActionTpl<SCALAR_T> scaled_action = action.scaled(output_scaler.getValue());
    q_des_feed_forward += scaled_action.kp * scaled_action.q_des;
    v_des_feed_forward += scaled_action.kd * scaled_action.v_des;
    kp_feedback += scaled_action.kp;
    kd_feedback += scaled_action.kd;
    feed_forward_effort += scaled_action.feed_forward_effort;
  }

  // Clears all accumulated terms back to zero.
  void reset() { *this = JointAccumulactionTpl{}; }

  SCALAR_T getTotalFeedbackTorque(SCALAR_T q, SCALAR_T v) const {
    return (q_des_feed_forward - kp_feedback * q + v_des_feed_forward - kd_feedback * v + feed_forward_effort);
  }

  // Recovers the (kp, q_des) form the motor firmware and the simulator drivers consume, by
  // undoing the kp * q_des products accumulated above.
  //
  // The gains are floored before dividing because a joint driven purely by feed-forward effort
  // — inverse dynamics, say — accumulates no stiffness at all, and a bare 0 / 0 yields a NaN
  // q_des. That NaN is not harmless: getTotalFeedbackTorque multiplies q_des by kp, and 0 * NaN
  // is NaN rather than 0, so the whole commanded torque becomes NaN and the physics diverges on
  // the first tick. Zero stiffness makes q_des a don't-care (it is multiplied by kp = 0), so
  // any finite value is equivalent and the floor picks 0.
  JointFeedbackActionTpl<SCALAR_T> toJointFeedbackAction() const {
    JointFeedbackActionTpl<SCALAR_T> action;
    action.q_des = q_des_feed_forward / (kp_feedback + kGainFloor);
    action.v_des = v_des_feed_forward / (kd_feedback + kGainFloor);
    action.kp = kp_feedback;
    action.kd = kd_feedback;
    action.feed_forward_effort = feed_forward_effort;
    return action;
  }
};

template <typename SCALAR_T>
class RobotJointFeedbackActionTpl : public JointIdMap<JointFeedbackActionTpl<SCALAR_T>, SCALAR_T> {
  using Base = JointIdMap<JointFeedbackActionTpl<SCALAR_T>, SCALAR_T>;

 public:
  explicit RobotJointFeedbackActionTpl(std::size_t joint_count) : Base(joint_count) {}
  explicit RobotJointFeedbackActionTpl(const RobotDescription& robot_description) : Base(robot_description) {}
};

template <typename SCALAR_T>
class RobotJointAccumulactionTpl : public JointIdMap<JointAccumulactionTpl<SCALAR_T>, SCALAR_T> {
  using Base = JointIdMap<JointAccumulactionTpl<SCALAR_T>, SCALAR_T>;

 public:
  explicit RobotJointAccumulactionTpl(std::size_t joint_count) : Base(joint_count) {}
  explicit RobotJointAccumulactionTpl(const RobotDescription& robot_description) : Base(robot_description) {}

  // Zeroes every joint's accumulation so the next control step starts fresh.
  void reset() {
    for (JointAccumulactionTpl<SCALAR_T>& joint_accumulaction : *this) {
      joint_accumulaction.reset();
    }
  }

  // Collapses the accumulated feedback law into one feedback action per joint, written into
  // out (which must be sized to the same robot description). Returned via out-parameter
  // because RobotJointFeedbackAction is not copy/move-constructible.
  void toRobotJointFeedbackAction(RobotJointFeedbackActionTpl<SCALAR_T>& out) const {
    for (size_t i = 0; i < this->size(); ++i) {
      out[i] = (*this)[i].toJointFeedbackAction();
    }
  }
};

using JointFeedbackAction = JointFeedbackActionTpl<scalar_t>;
using JointAccumulaction = JointAccumulactionTpl<scalar_t>;
using RobotJointFeedbackAction = RobotJointFeedbackActionTpl<scalar_t>;
using RobotJointAccumulaction = RobotJointAccumulactionTpl<scalar_t>;

}  // namespace motorium::model
