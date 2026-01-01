#pragma once

#include <motorium_model/FixedIDArray.h>
#include <motorium_model/RobotDescription.h>

#include <functional>
#include <vector>

/**
 * Convert a sequence of joint indices into an Eigen vector of scalar values.
 *
 * Builds a vector whose i-th element is the scalar extracted from this map for
 * the joint index at joint_ids[i]. If a joint does not have a usable value,
 * the corresponding element is set to default_value.
 *
 * @param joint_ids Span of joint indices to include, in the desired output order.
 * @param value_extractor Callable that extracts a `scalar_t` from a stored `T` value.
 * @param default_value Value to use for output entries when a joint has no usable value.
 * @returns `vector_t` whose size equals `joint_ids.size()` and whose elements are the extracted or defaulted scalar values.
 */
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