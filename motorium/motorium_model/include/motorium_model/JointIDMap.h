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

#include <motorium_model/FixedIDArray.h>
#include <motorium_model/RobotDescription.h>

#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

namespace motorium::model {

template <typename T, typename SCALAR_T = scalar_t>
class JointIdMap : public FixedIDArray<T> {
 public:
  explicit JointIdMap(std::size_t joint_count) : FixedIDArray<T>(joint_count) {}
  explicit JointIdMap(const RobotDescription& robot_description) : JointIdMap(robot_description.getNumJoints()) {}

  JointIdMap() = delete;

  // Fill a caller-owned, pre-sized vector (real-time safe, no allocation).
  void fillVector(std::span<const joint_index_t> joint_ids,
                  VECTOR_T<SCALAR_T>& out,
                  IDMapExtractor<T, SCALAR_T> auto value_extractor,
                  SCALAR_T default_value = std::numeric_limits<SCALAR_T>::quiet_NaN()) const {
    this->fillEigenVector(joint_ids, out, value_extractor, default_value);
  }

  void fromVector(std::span<const joint_index_t> joint_ids, const VECTOR_T<SCALAR_T>& vector, IDMapInserter<T, SCALAR_T> auto inserter) {
    this->fromEigenVector(joint_ids, vector, inserter);
  }
};

}  // namespace motorium::model
