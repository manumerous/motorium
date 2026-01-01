#pragma once

#include <cassert>
#include <cstring>

#include "absl/log/check.h"

/**
 * Evaluates `cond` and terminates the program if the condition is false.
 *
 * @param cond Condition to check; if it evaluates to false the program will
 *             be terminated with an explanatory message.
 */

/**
 * Debug-only check: evaluates `cond` and terminates the program if the
 * condition is false when debug checks are enabled.
 *
 * @param cond Condition to check; has effect only in builds that enable
 *             debug checks and will terminate the program if false in those builds.
 */

/**
 * Evaluates `cond` and, if the condition is false, logs diagnostic information
 * including system error details and then terminates the program.
 *
 * @param cond Condition to check; if it evaluates to false the macro logs
 *             additional system error information and terminates the program.
 */
namespace motorium {

#define MT_CHECK(cond) CHECK(cond)    // Always Evaluated
#define MT_DCHECK(cond) DCHECK(cond)  // Debug Check
#define MT_PCHECK(cond) PCHECK(cond)  // Prints extra information

}  // namespace motorium