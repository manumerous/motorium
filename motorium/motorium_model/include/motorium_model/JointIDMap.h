#pragma once

#include <motorium_model/FixedIDArray.h>
#include <motorium_model/RobotDescription.h>

#include <functional>
#include <vector>

namespace motorium::model {

template <typename T>
class JointIdMap : public FixedIDArray<T> {
 public:
  explicit JointIdMap(const RobotDescription& robotDescription) : FixedIDArray<T>(robotDescription.getNumJoints()) {}

  JointIdMap() = delete;

  vector_t toVector(std::span<const joint_index_t> joint_ids,
                    IDMapExtractor<T, scalar_t> auto value_extractor,
                    scalar_t default_value = std::numeric_limits<scalar_t>::quiet_NaN()) const {
    return this->toEigenVector(joint_ids, value_extractor, default_value);
  }

 private:
};

}  // namespace motorium::model
