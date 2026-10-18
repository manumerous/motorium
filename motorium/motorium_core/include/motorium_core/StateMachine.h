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

#include <atomic>
#include <string_view>

#include <motorium_core/Check.h>
#include <magic_enum/magic_enum.hpp>

namespace motorium::core {

// Thread-safe, lock-free state machine. The transition table is a compile-time function
// IsLegalTransition(from, to); illegal transitions abort via MT_CHECK.
//
// The state is a single std::atomic, so reads are plain atomic loads and transitions are
// compare-and-swap operations: no thread can ever block another, which keeps getState() safe to
// call from real-time loops. If the state ever has to be updated together with other data (e.g. a
// fault reason), this needs to become a mutex again.
//
// Meant to be held as a (private) member by the class whose lifecycle it tracks,
// so that class alone decides who may trigger which transition.
template <typename StateEnum, bool (*IsLegalTransition)(StateEnum, StateEnum)>
class StateMachine final {
  static_assert(std::atomic<StateEnum>::is_always_lock_free, "StateMachine requires a lock-free atomic state");

 public:
  explicit StateMachine(StateEnum initial_state) : state_(initial_state) {}

  StateEnum getState() const { return state_.load(); }

  // Returns a view into static storage (magic_enum::enum_name guarantees program lifetime).
  std::string_view stateToString() const { return magic_enum::enum_name(getState()); }

  // Transitions to `next`, aborting if the transition is illegal from the current state. If another
  // thread changes the state concurrently, legality is re-checked against the new state.
  void requestTransitionTo(StateEnum next) {
    StateEnum current = state_.load();
    do {
      checkLegal(current, next);
    } while (!state_.compare_exchange_weak(current, next));  // on failure, `current` is refreshed
  }

  // Atomically transitions to `next` only if the current state is still `expected`, and returns
  // whether it did. Use this when the state may change concurrently between reading and writing it,
  // e.g. to avoid overwriting a FAULT reported from another thread. Aborts if expected -> next is
  // illegal, regardless of the current state, since that is a bug in the caller.
  bool tryTransition(StateEnum expected, StateEnum next) {
    checkLegal(expected, next);
    return state_.compare_exchange_strong(expected, next);
  }

 private:
  static void checkLegal(StateEnum from, StateEnum to) {
    MT_CHECK(IsLegalTransition(from, to)) << "Illegal state transition: " << magic_enum::enum_name(from) << " -> "
                                          << magic_enum::enum_name(to);
  }

  std::atomic<StateEnum> state_;
};

}  // namespace motorium::core
