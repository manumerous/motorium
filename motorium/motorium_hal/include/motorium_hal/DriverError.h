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

#include <cstdint>
#include <expected>
#include <format>
#include <optional>
#include <ostream>
#include <string_view>

#include <motorium_core/Types.h>

#include <magic_enum/magic_enum.hpp>

namespace motorium::hal {

// Why a driver or HAL call failed. Runtime conditions only: programming errors (e.g. starting a
// driver that is not ready) still abort via MT_CHECK.
enum class DriverErrorCode : std::uint8_t {
  // Set by the framework (DriverBase / RobotHAL), never by drivers.
  NotRunning,         // Call made while the driver was not RUNNING.
  FaultedDuringCall,  // The driver faulted (e.g. from its own thread) while the call was running.
  HalLatched,         // The HAL is latched in FAULT after a safety stop; resetDrivers() is needed.

  // Set by drivers in their *Impl() hooks.
  CommTimeout,     // Bus/link timeout, or a reply missed the cycle.
  DataCorrupted,   // Bad checksum, or an implausible or corrupted reading.
  LimitViolation,  // A command outside the position, velocity or torque limits.
  HardwareFault,   // The device reports a fault (overtemperature, encoder error, ...).
  Internal,        // A broken invariant inside the driver (e.g. the simulation diverged).
};

// A failed driver or HAL call. A plain value type that never allocates, so errors can be created and
// passed around on the real-time path.
struct DriverError {
  DriverErrorCode code;
  // Human-readable detail. Must have static storage duration (a string literal), since the error may
  // outlive the scope that created it.
  std::string_view detail{};
  // The joint concerned, if the failure is specific to one joint.
  std::optional<joint_index_t> joint{};
  // Name of the driver that failed. Filled in by DriverBase, so drivers leave it empty. Refers to the
  // driver's own name and stays valid as long as the RobotHAL owning the driver lives.
  std::string_view driver{};
};

// Result of a driver or HAL call: either a value (nothing, for DriverResult<>) or a DriverError.
template <typename T = void>
using DriverResult = std::expected<T, DriverError>;

// Shorthand for returning an error from a function returning DriverResult, e.g.
//   return driverError(DriverErrorCode::CommTimeout, "CAN send timed out");
inline std::unexpected<DriverError> driverError(DriverErrorCode code,
                                                std::string_view detail,
                                                std::optional<joint_index_t> joint = std::nullopt) {
  return std::unexpected(DriverError{.code = code, .detail = detail, .joint = joint});
}

}  // namespace motorium::hal

// Formats as "<driver>: <code> (<detail>) [joint <n>]", leaving out the parts that are not set.
template <>
struct std::formatter<motorium::hal::DriverError> : std::formatter<std::string_view> {
  auto format(const motorium::hal::DriverError& error, std::format_context& ctx) const {
    auto out = ctx.out();
    if (!error.driver.empty()) out = std::format_to(out, "{}: ", error.driver);
    out = std::format_to(out, "{}", magic_enum::enum_name(error.code));
    if (!error.detail.empty()) out = std::format_to(out, " ({})", error.detail);
    if (error.joint) out = std::format_to(out, " [joint {}]", *error.joint);
    return out;
  }
};

namespace motorium::hal {

inline std::ostream& operator<<(std::ostream& os, const DriverError& error) {
  return os << std::format("{}", error);
}

}  // namespace motorium::hal
