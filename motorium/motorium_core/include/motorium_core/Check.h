#pragma once

#include "absl/log/check.h"

#define MT_CHECK(cond) CHECK(cond)    // Always Evaluated
#define MT_DCHECK(cond) DCHECK(cond)  // Debug Check
#define MT_PCHECK(cond) PCHECK(cond)  // Prints extra information
