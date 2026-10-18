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

#include <gtest/gtest.h>
#include <motorium_core/StateMachine.h>

#include <atomic>
#include <thread>
#include <vector>

using namespace motorium::core;

enum class TestState { IDLE, ACTIVE, PAUSED, FAULT };

// Accepts every transition so multiple threads can write concurrently without
// hitting the MT_CHECK. Used to stress the locking logic in isolation.
bool allowAll(TestState, TestState) {
  return true;
}

bool isLegalTestTransition(TestState from, TestState to) {
  if (to == TestState::FAULT) return from != TestState::FAULT;
  switch (from) {
    case TestState::IDLE:
      return to == TestState::ACTIVE;
    case TestState::ACTIVE:
      return to == TestState::PAUSED || to == TestState::IDLE;
    case TestState::PAUSED:
      return to == TestState::ACTIVE;
    case TestState::FAULT:
      return to == TestState::IDLE;
    default:
      return false;
  }
}

using StressMachine = StateMachine<TestState, allowAll>;
using TestMachine = StateMachine<TestState, isLegalTestTransition>;

TEST(StateMachineTest, InitialState) {
  TestMachine m(TestState::IDLE);
  EXPECT_EQ(m.getState(), TestState::IDLE);
}

TEST(StateMachineTest, LegalTransition) {
  TestMachine m(TestState::IDLE);
  m.requestTransitionTo(TestState::ACTIVE);
  EXPECT_EQ(m.getState(), TestState::ACTIVE);
}

TEST(StateMachineTest, LegalTransitionSequence) {
  TestMachine m(TestState::IDLE);
  m.requestTransitionTo(TestState::ACTIVE);
  m.requestTransitionTo(TestState::PAUSED);
  m.requestTransitionTo(TestState::ACTIVE);
  EXPECT_EQ(m.getState(), TestState::ACTIVE);
}

TEST(StateMachineTest, FaultReachableFromAnyNonFaultState) {
  for (TestState from : {TestState::IDLE, TestState::ACTIVE, TestState::PAUSED}) {
    TestMachine m(TestState::IDLE);
    if (from != TestState::IDLE) m.requestTransitionTo(TestState::ACTIVE);
    if (from == TestState::PAUSED) m.requestTransitionTo(TestState::PAUSED);
    m.requestTransitionTo(TestState::FAULT);
    EXPECT_EQ(m.getState(), TestState::FAULT);
  }
}

TEST(StateMachineTest, IllegalTransitionKills) {
  TestMachine m(TestState::IDLE);
  EXPECT_DEATH(m.requestTransitionTo(TestState::PAUSED), "Illegal state transition");
}

TEST(StateMachineTest, StateToString) {
  TestMachine m(TestState::IDLE);
  EXPECT_EQ(m.stateToString(), "IDLE");
  m.requestTransitionTo(TestState::ACTIVE);
  EXPECT_EQ(m.stateToString(), "ACTIVE");
}

TEST(StateMachineTest, ConcurrentReadsAreSafe) {
  TestMachine m(TestState::IDLE);
  m.requestTransitionTo(TestState::ACTIVE);

  std::vector<std::thread> threads;
  for (int i = 0; i < 8; ++i) {
    threads.emplace_back([&m]() {
      for (int j = 0; j < 10000; ++j) {
        (void)m.getState();
      }
    });
  }
  for (auto& t : threads) t.join();
  EXPECT_EQ(m.getState(), TestState::ACTIVE);
}

TEST(StateMachineTest, TryTransitionSucceedsFromExpectedState) {
  TestMachine m(TestState::IDLE);
  EXPECT_TRUE(m.tryTransition(TestState::IDLE, TestState::ACTIVE));
  EXPECT_EQ(m.getState(), TestState::ACTIVE);
}

TEST(StateMachineTest, TryTransitionIsNoOpFromOtherState) {
  TestMachine m(TestState::IDLE);
  m.requestTransitionTo(TestState::FAULT);
  // A FAULT that arrived in the meantime must not be overwritten.
  EXPECT_FALSE(m.tryTransition(TestState::IDLE, TestState::ACTIVE));
  EXPECT_EQ(m.getState(), TestState::FAULT);
}

TEST(StateMachineTest, TryTransitionKillsOnIllegalTransition) {
  TestMachine m(TestState::IDLE);
  EXPECT_DEATH(m.tryTransition(TestState::IDLE, TestState::PAUSED), "Illegal state transition");
}

TEST(StateMachineTest, ConcurrentTryTransitionsSucceedExactlyOnce) {
  StressMachine m(TestState::IDLE);
  std::atomic<int> successes{0};

  std::vector<std::thread> threads;
  for (int i = 0; i < 8; ++i) {
    threads.emplace_back([&m, &successes]() {
      if (m.tryTransition(TestState::IDLE, TestState::ACTIVE)) successes++;
    });
  }
  for (auto& t : threads) t.join();
  EXPECT_EQ(successes.load(), 1);
  EXPECT_EQ(m.getState(), TestState::ACTIVE);
}
