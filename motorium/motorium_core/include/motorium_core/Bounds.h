#pragma once

#include <algorithm>
#include <limits>

#include <motorium_core/Types.h>

/**
 * Represents a numeric closed interval [min, max] with utility operations.
 */

/**
 * Construct a Bounds with explicit lower and upper limits.
 * @param minimum Lower bound of the interval.
 * @param maximum Upper bound of the interval.
 */

/**
 * Construct an unbounded Bounds spanning negative to positive infinity.
 */

/**
 * Lower bound of the interval.
 */

/**
 * Upper bound of the interval.
 */

/**
 * Clamp a value to the interval [min, max].
 * @param val Value to clamp.
 * @returns The value constrained to lie within [min, max].
 */

/**
 * Compute how far a value lies outside the interval.
 * @param val Value to evaluate.
 * @returns Positive amount by which `val` exceeds `max`, negative amount by which `val` is below `min`, or `0` if `val` is inside the interval.
 */

/**
 * Stream representation of Bounds in the form "Bounds { min: <min>, max: <max> }".
 * @param os Output stream to write to.
 * @param bounds Bounds instance to format.
 * @returns Reference to `os`.
 */
namespace motorium {

struct Bounds {
  Bounds(scalar_t minimum, scalar_t maximum) : min(minimum), max(maximum){};
  Bounds() : min(-std::numeric_limits<scalar_t>::infinity()), max(std::numeric_limits<scalar_t>::infinity()){};
  scalar_t min;
  scalar_t max;

  scalar_t clamp(scalar_t val) const { return std::clamp(val, min, max); }
  scalar_t violation(scalar_t val) const {
    if (val > max) return val - max;
    if (val < min) return val - min;
    return 0;
  }

  friend std::ostream& operator<<(std::ostream& os, const Bounds& bounds);
};

inline std::ostream& operator<<(std::ostream& os, const Bounds& bounds) {
  os << "Bounds { " << "min: " << bounds.min << ", max: " << bounds.max << " }";
  return os;
}

}  // namespace motorium