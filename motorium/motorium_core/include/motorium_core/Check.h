#pragma once

#include <cassert>
#include <cstring>

#include "absl/log/check.h"

namespace motorium {

#define MT_CHECK(cond) CHECK(cond)    // Always Evaluated
#define MT_DCHECK(cond) DCHECK(cond)  // Debug Check
#define MT_PCHECK(cond) PCHECK(cond)  // Prints extra information

}  // namespace motorium