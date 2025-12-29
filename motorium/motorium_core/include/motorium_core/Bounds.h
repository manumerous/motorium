#pragma once

#include <algorithm>
#include <limits>

#include <motorium_core/Types.h>

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