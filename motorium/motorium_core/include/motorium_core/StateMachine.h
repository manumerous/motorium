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

#include <absl/synchronization/mutex.h>
#include <string_view>

#include <motorium_core/Check.h>
#include <magic_enum/magic_enum.hpp>

namespace motorium::core {

// Thread-safe state machine base. Subclasses provide the transition table via
// isLegalTransition() and drive transitions through the protected transitionTo().
template <typename StateEnum>
class StateMachine {
 public:
  explicit StateMachine(StateEnum initial_state) : state_(initial_state) {}
  virtual ~StateMachine() = default;

  StateEnum getState() const {
    absl::ReaderMutexLock lock(&mu_);
    return state_;
  }

  // Returns a view into static storage (magic_enum::enum_name guarantees program lifetime).
  std::string_view stateToString() const { return magic_enum::enum_name(getState()); }

  void requestTransitionTo(StateEnum next) {
    absl::MutexLock lock(&mu_);
    MT_CHECK(isLegalTransition(state_, next)) << "Illegal state transition: " << magic_enum::enum_name(state_) << " -> "
                                              << magic_enum::enum_name(next);
    state_ = next;
  }

 protected:
  virtual bool isLegalTransition(StateEnum from, StateEnum to) const = 0;

 private:
  mutable absl::Mutex mu_;
  StateEnum state_ ABSL_GUARDED_BY(mu_);  // Clang ignores the ABSL_GUARDED_BY annotation in template classes -.-
};

}  // namespace motorium::core
