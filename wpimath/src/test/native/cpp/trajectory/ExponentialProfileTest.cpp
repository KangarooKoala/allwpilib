// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/trajectory/ExponentialProfile.h"  // NOLINT(build/include_order)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "frc/controller/SimpleMotorFeedforward.h"
#include "frc/units.h"

static constexpr auto kDt = 10.0 * mp::ms;
static constexpr auto kV = 2.5629 * mp::V / (mp::m / mp::s);
static constexpr auto kA = 0.43277 * mp::V / (mp::m / mp::s2);

#define EXPECT_NEAR_UNITS(val1, val2, eps) EXPECT_LE(mp::abs(val1 - val2), eps)

#define EXPECT_LT_OR_NEAR_UNITS(val1, val2, eps) \
  if (val1 <= val2) {                            \
    EXPECT_LE(val1, val2);                       \
  } else {                                       \
    EXPECT_NEAR_UNITS(val1, val2, eps);          \
  }

frc::ExponentialProfile<mp::m, mp::V>::State CheckDynamics(
    frc::ExponentialProfile<mp::m, mp::V> profile,
    frc::ExponentialProfile<mp::m, mp::V>::Constraints constraints,
    frc::SimpleMotorFeedforward<mp::m> feedforward,
    frc::ExponentialProfile<mp::m, mp::V>::State current,
    frc::ExponentialProfile<mp::m, mp::V>::State goal) {
  auto next = profile.Calculate(kDt, current, goal);
  auto signal = feedforward.Calculate(current.velocity, next.velocity);

  EXPECT_LE(mp::abs(signal), (constraints.maxInput + 1e-9 * mp::V));

  return next;
}

TEST(ExponentialProfileTest, ReachesGoal) {
  frc::ExponentialProfile<mp::m, mp::V>::Constraints constraints{
      12.0 * mp::V, -kV / kA, 1 / kA};
  frc::ExponentialProfile<mp::m, mp::V> profile{constraints};
  frc::SimpleMotorFeedforward<mp::m> feedforward{
      0.0 * mp::V, 2.5629 * mp::V / (mp::m / mp::s),
      0.43277 * mp::V / (mp::m / mp::s2), kDt};
  frc::ExponentialProfile<mp::m, mp::V>::State goal{10.0 * mp::m,
                                                    0.0 * mp::m / mp::s};
  frc::ExponentialProfile<mp::m, mp::V>::State state{0.0 * mp::m,
                                                     0.0 * mp::m / mp::s};

  for (int i = 0; i < 450; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
  }
  EXPECT_EQ(state, goal);
}

// Tests that decreasing the maximum velocity in the middle when it is already
// moving faster than the new max is handled correctly
TEST(ExponentialProfileTest, PosContinuousUnderVelChange) {
  frc::ExponentialProfile<mp::m, mp::V>::Constraints constraints{
      12.0 * mp::V, -kV / kA, 1 / kA};
  frc::ExponentialProfile<mp::m, mp::V> profile{constraints};
  frc::SimpleMotorFeedforward<mp::m> feedforward{
      0.0 * mp::V, 2.5629 * mp::V / (mp::m / mp::s),
      0.43277 * mp::V / (mp::m / mp::s2), kDt};
  frc::ExponentialProfile<mp::m, mp::V>::State goal{10.0 * mp::m,
                                                    0.0 * mp::m / mp::s};
  frc::ExponentialProfile<mp::m, mp::V>::State state{0.0 * mp::m,
                                                     0.0 * mp::m / mp::s};

  for (int i = 0; i < 300; ++i) {
    if (i == 150) {
      constraints.maxInput = 9.0 * mp::V;
      profile = frc::ExponentialProfile<mp::m, mp::V>{constraints};
    }

    state = CheckDynamics(profile, constraints, feedforward, state, goal);
  }
  EXPECT_EQ(state, goal);
}

// Tests that decreasing the maximum velocity in the middle when it is already
// moving faster than the new max is handled correctly
TEST(ExponentialProfileTest, PosContinuousUnderVelChangeBackward) {
  frc::ExponentialProfile<mp::m, mp::V>::Constraints constraints{
      12.0 * mp::V, -kV / kA, 1 / kA};
  frc::ExponentialProfile<mp::m, mp::V> profile{constraints};
  frc::SimpleMotorFeedforward<mp::m> feedforward{
      0.0 * mp::V, 2.5629 * mp::V / (mp::m / mp::s),
      0.43277 * mp::V / (mp::m / mp::s2), kDt};
  frc::ExponentialProfile<mp::m, mp::V>::State goal{-10.0 * mp::m,
                                                    0.0 * mp::m / mp::s};
  frc::ExponentialProfile<mp::m, mp::V>::State state{0.0 * mp::m,
                                                     0.0 * mp::m / mp::s};

  for (int i = 0; i < 300; ++i) {
    if (i == 150) {
      constraints.maxInput = 9.0 * mp::V;
      profile = frc::ExponentialProfile<mp::m, mp::V>{constraints};
    }

    state = CheckDynamics(profile, constraints, feedforward, state, goal);
  }
  EXPECT_EQ(state, goal);
}

// There is some somewhat tricky code for dealing with going backwards
TEST(ExponentialProfileTest, Backwards) {
  frc::ExponentialProfile<mp::m, mp::V>::Constraints constraints{
      12.0 * mp::V, -kV / kA, 1 / kA};
  frc::ExponentialProfile<mp::m, mp::V> profile{constraints};
  frc::SimpleMotorFeedforward<mp::m> feedforward{
      0.0 * mp::V, 2.5629 * mp::V / (mp::m / mp::s),
      0.43277 * mp::V / (mp::m / mp::s2), kDt};
  frc::ExponentialProfile<mp::m, mp::V>::State goal{-10.0 * mp::m,
                                                    0.0 * mp::m / mp::s};
  frc::ExponentialProfile<mp::m, mp::V>::State state;

  for (int i = 0; i < 400; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
  }
  EXPECT_EQ(state, goal);
}

TEST(ExponentialProfileTest, SwitchGoalInMiddle) {
  frc::ExponentialProfile<mp::m, mp::V>::Constraints constraints{
      12.0 * mp::V, -kV / kA, 1 / kA};
  frc::ExponentialProfile<mp::m, mp::V> profile{constraints};
  frc::SimpleMotorFeedforward<mp::m> feedforward{
      0.0 * mp::V, 2.5629 * mp::V / (mp::m / mp::s),
      0.43277 * mp::V / (mp::m / mp::s2), kDt};
  frc::ExponentialProfile<mp::m, mp::V>::State goal{-10.0 * mp::m,
                                                    0.0 * mp::m / mp::s};
  frc::ExponentialProfile<mp::m, mp::V>::State state{0.0 * mp::m,
                                                     0.0 * mp::m / mp::s};

  for (int i = 0; i < 50; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
  }
  EXPECT_NE(state, goal);

  goal = {0.0 * mp::m, 0.0 * mp::m / mp::s};
  for (int i = 0; i < 100; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
  }
  EXPECT_EQ(state, goal);
}

// Checks to make sure that it hits top speed on long trajectories
TEST(ExponentialProfileTest, TopSpeed) {
  frc::ExponentialProfile<mp::m, mp::V>::Constraints constraints{
      12.0 * mp::V, -kV / kA, 1 / kA};
  frc::ExponentialProfile<mp::m, mp::V> profile{constraints};
  frc::SimpleMotorFeedforward<mp::m> feedforward{
      0.0 * mp::V, 2.5629 * mp::V / (mp::m / mp::s),
      0.43277 * mp::V / (mp::m / mp::s2), kDt};
  frc::ExponentialProfile<mp::m, mp::V>::State goal{40.0 * mp::m,
                                                    0.0 * mp::m / mp::s};
  frc::ExponentialProfile<mp::m, mp::V>::State state;

  mp::quantity<mp::m / mp::s> maxSpeed = 0.0 * mp::m / mp::s;

  for (int i = 0; i < 900; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
    maxSpeed = std::max(state.velocity, maxSpeed);
  }

  EXPECT_NEAR_UNITS(constraints.MaxVelocity(), maxSpeed, 1e-5 * mp::m / mp::s);
  EXPECT_EQ(state, goal);
}

// Checks to make sure that it hits top speed on long trajectories
TEST(ExponentialProfileTest, TopSpeedBackward) {
  frc::ExponentialProfile<mp::m, mp::V>::Constraints constraints{
      12.0 * mp::V, -kV / kA, 1 / kA};
  frc::ExponentialProfile<mp::m, mp::V> profile{constraints};
  frc::SimpleMotorFeedforward<mp::m> feedforward{
      0.0 * mp::V, 2.5629 * mp::V / (mp::m / mp::s),
      0.43277 * mp::V / (mp::m / mp::s2), kDt};
  frc::ExponentialProfile<mp::m, mp::V>::State goal{-40.0 * mp::m,
                                                    0.0 * mp::m / mp::s};
  frc::ExponentialProfile<mp::m, mp::V>::State state;

  mp::quantity<mp::m / mp::s> maxSpeed = 0.0 * mp::m / mp::s;

  for (int i = 0; i < 900; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
    maxSpeed = std::min(state.velocity, maxSpeed);
  }

  EXPECT_NEAR_UNITS(-constraints.MaxVelocity(), maxSpeed, 1e-5 * mp::m / mp::s);
  EXPECT_EQ(state, goal);
}

// Checks to make sure that it hits top speed on long trajectories
TEST(ExponentialProfileTest, HighInitialSpeed) {
  frc::ExponentialProfile<mp::m, mp::V>::Constraints constraints{
      12.0 * mp::V, -kV / kA, 1 / kA};
  frc::ExponentialProfile<mp::m, mp::V> profile{constraints};
  frc::SimpleMotorFeedforward<mp::m> feedforward{
      0.0 * mp::V, 2.5629 * mp::V / (mp::m / mp::s),
      0.43277 * mp::V / (mp::m / mp::s2), kDt};
  frc::ExponentialProfile<mp::m, mp::V>::State goal{40.0 * mp::m,
                                                    0.0 * mp::m / mp::s};
  frc::ExponentialProfile<mp::m, mp::V>::State state{0.0 * mp::m,
                                                     8.0 * mp::m / mp::s};

  for (int i = 0; i < 900; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
  }

  EXPECT_EQ(state, goal);
}

// Checks to make sure that it hits top speed on long trajectories
TEST(ExponentialProfileTest, HighInitialSpeedBackward) {
  frc::ExponentialProfile<mp::m, mp::V>::Constraints constraints{
      12.0 * mp::V, -kV / kA, 1 / kA};
  frc::ExponentialProfile<mp::m, mp::V> profile{constraints};
  frc::SimpleMotorFeedforward<mp::m> feedforward{
      0.0 * mp::V, 2.5629 * mp::V / (mp::m / mp::s),
      0.43277 * mp::V / (mp::m / mp::s2), kDt};
  frc::ExponentialProfile<mp::m, mp::V>::State goal{-40.0 * mp::m,
                                                    0.0 * mp::m / mp::s};
  frc::ExponentialProfile<mp::m, mp::V>::State state{0.0 * mp::m,
                                                     -8.0 * mp::m / mp::s};

  for (int i = 0; i < 900; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
  }

  EXPECT_EQ(state, goal);
}

TEST(ExponentialProfileTest, TestHeuristic) {
  frc::ExponentialProfile<mp::m, mp::V>::Constraints constraints{
      12.0 * mp::V, -kV / kA, 1 / kA};
  frc::ExponentialProfile<mp::m, mp::V> profile{constraints};
  std::vector<
      std::tuple<frc::ExponentialProfile<mp::m, mp::V>::State,  // initial
                 frc::ExponentialProfile<mp::m, mp::V>::State,  // goal
                 frc::ExponentialProfile<mp::m, mp::V>::State>  // inflection
                                                                // point
      >
      testCases{
          // red > green and purple => always positive => false
          {{0.0 * mp::m, -4.0 * mp::m / mp::s},
           {0.75 * mp::m, -4.0 * mp::m / mp::s},
           {1.3758 * mp::m, 4.4304 * mp::m / mp::s}},
          {{0.0 * mp::m, -4.0 * mp::m / mp::s},
           {1.4103 * mp::m, 4.0 * mp::m / mp::s},
           {1.3758 * mp::m, 4.4304 * mp::m / mp::s}},
          {{0.6603 * mp::m, 4.0 * mp::m / mp::s},
           {0.75 * mp::m, -4.0 * mp::m / mp::s},
           {1.3758 * mp::m, 4.4304 * mp::m / mp::s}},
          {{0.6603 * mp::m, 4.0 * mp::m / mp::s},
           {1.4103 * mp::m, 4.0 * mp::m / mp::s},
           {1.3758 * mp::m, 4.4304 * mp::m / mp::s}},

          // purple > red > green => positive if v0 < 0 => c && !d && a
          {{0.0 * mp::m, -4.0 * mp::m / mp::s},
           {0.5 * mp::m, -2.0 * mp::m / mp::s},
           {0.4367 * mp::m, 3.7217 * mp::m / mp::s}},
          {{0.0 * mp::m, -4.0 * mp::m / mp::s},
           {0.546 * mp::m, 2.0 * mp::m / mp::s},
           {0.4367 * mp::m, 3.7217 * mp::m / mp::s}},
          {{0.6603 * mp::m, 4.0 * mp::m / mp::s},
           {0.5 * mp::m, -2.0 * mp::m / mp::s},
           {0.5560 * mp::m, -2.9616 * mp::m / mp::s}},
          {{0.6603 * mp::m, 4.0 * mp::m / mp::s},
           {0.546 * mp::m, 2.0 * mp::m / mp::s},
           {0.5560 * mp::m, -2.9616 * mp::m / mp::s}},

          // red < green and purple => always negative => true => !c && !d
          {{0.0 * mp::m, -4.0 * mp::m / mp::s},
           {-0.75 * mp::m, -4.0 * mp::m / mp::s},
           {-0.7156 * mp::m, -4.4304 * mp::m / mp::s}},
          {{0.0 * mp::m, -4.0 * mp::m / mp::s},
           {-0.0897 * mp::m, 4.0 * mp::m / mp::s},
           {-0.7156 * mp::m, -4.4304 * mp::m / mp::s}},
          {{0.6603 * mp::m, 4.0 * mp::m / mp::s},
           {-0.75 * mp::m, -4.0 * mp::m / mp::s},
           {-0.7156 * mp::m, -4.4304 * mp::m / mp::s}},
          {{0.6603 * mp::m, 4.0 * mp::m / mp::s},
           {-0.0897 * mp::m, 4.0 * mp::m / mp::s},
           {-0.7156 * mp::m, -4.4304 * mp::m / mp::s}},

          // green > red > purple => positive if vf < 0 => !c && d && b
          {{0.0 * mp::m, -4.0 * mp::m / mp::s},
           {-0.5 * mp::m, -4.5 * mp::m / mp::s},
           {1.095 * mp::m, 4.314 * mp::m / mp::s}},
          {{0.0 * mp::m, -4.0 * mp::m / mp::s},
           {1.0795 * mp::m, 4.5 * mp::m / mp::s},
           {-0.5122 * mp::m, -4.351 * mp::m / mp::s}},
          {{0.6603 * mp::m, 4.0 * mp::m / mp::s},
           {-0.5 * mp::m, -4.5 * mp::m / mp::s},
           {1.095 * mp::m, 4.314 * mp::m / mp::s}},
          {{0.6603 * mp::m, 4.0 * mp::m / mp::s},
           {1.0795 * mp::m, 4.5 * mp::m / mp::s},
           {-0.5122 * mp::m, -4.351 * mp::m / mp::s}},

          // tests for initial velocity > V/kV
          {{0.0 * mp::m, -8.0 * mp::m / mp::s},
           {0.0 * mp::m, 0.0 * mp::m / mp::s},
           {-0.1384 * mp::m, 3.342 * mp::m / mp::s}},
          {{0.0 * mp::m, -8.0 * mp::m / mp::s},
           {-1.0 * mp::m, 0.0 * mp::m / mp::s},
           {-0.562 * mp::m, -6.792 * mp::m / mp::s}},
          {{0.0 * mp::m, 8.0 * mp::m / mp::s},
           {1.0 * mp::m, 0.0 * mp::m / mp::s},
           {0.562 * mp::m, 6.792 * mp::m / mp::s}},
          {{0.0 * mp::m, 8.0 * mp::m / mp::s},
           {-1.0 * mp::m, 0.0 * mp::m / mp::s},
           {-0.785 * mp::m, -4.346 * mp::m / mp::s}},
      };

  for (auto& testCase : testCases) {
    auto state = profile.CalculateInflectionPoint(std::get<0>(testCase),
                                                  std::get<1>(testCase));
    EXPECT_NEAR_UNITS(std::get<2>(testCase).position, state.position,
                      1e-3 * mp::m);
    EXPECT_NEAR_UNITS(std::get<2>(testCase).velocity, state.velocity,
                      1e-3 * mp::m / mp::s);
  }
}

TEST(ExponentialProfileTest, TimingToCurrent) {
  frc::ExponentialProfile<mp::m, mp::V>::Constraints constraints{
      12.0 * mp::V, -kV / kA, 1 / kA};
  frc::ExponentialProfile<mp::m, mp::V> profile{constraints};
  frc::SimpleMotorFeedforward<mp::m> feedforward{
      0.0 * mp::V, 2.5629 * mp::V / (mp::m / mp::s),
      0.43277 * mp::V / (mp::m / mp::s2), kDt};
  frc::ExponentialProfile<mp::m, mp::V>::State goal{2.0 * mp::m,
                                                    0.0 * mp::m / mp::s};
  frc::ExponentialProfile<mp::m, mp::V>::State state{0.0 * mp::m,
                                                     0.0 * mp::m / mp::s};

  for (int i = 0; i < 900; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
    EXPECT_NEAR_UNITS(profile.TimeLeftUntil(state, state), 0.0 * mp::s,
                      2e-2 * mp::s);
  }

  EXPECT_EQ(state, goal);
}

TEST(ExponentialProfileTest, TimingToGoal) {
  frc::ExponentialProfile<mp::m, mp::V>::Constraints constraints{
      12.0 * mp::V, -kV / kA, 1 / kA};
  frc::ExponentialProfile<mp::m, mp::V> profile{constraints};
  frc::SimpleMotorFeedforward<mp::m> feedforward{
      0.0 * mp::V, 2.5629 * mp::V / (mp::m / mp::s),
      0.43277 * mp::V / (mp::m / mp::s2), kDt};
  frc::ExponentialProfile<mp::m, mp::V>::State goal{2.0 * mp::m,
                                                    0.0 * mp::m / mp::s};
  frc::ExponentialProfile<mp::m, mp::V>::State state{0.0 * mp::m,
                                                     0.0 * mp::m / mp::s};

  auto prediction = profile.TimeLeftUntil(state, goal);
  auto reachedGoal = false;

  for (int i = 0; i < 900; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
    if (!reachedGoal && state == goal) {
      EXPECT_NEAR_UNITS(prediction, i * 10.0 * mp::ms, 250.0 * mp::ms);
      reachedGoal = true;
    }
  }

  EXPECT_EQ(state, goal);
}

TEST(ExponentialProfileTest, TimingToNegativeGoal) {
  frc::ExponentialProfile<mp::m, mp::V>::Constraints constraints{
      12.0 * mp::V, -kV / kA, 1 / kA};
  frc::ExponentialProfile<mp::m, mp::V> profile{constraints};
  frc::SimpleMotorFeedforward<mp::m> feedforward{
      0.0 * mp::V, 2.5629 * mp::V / (mp::m / mp::s),
      0.43277 * mp::V / (mp::m / mp::s2), kDt};
  frc::ExponentialProfile<mp::m, mp::V>::State goal{-2.0 * mp::m,
                                                    0.0 * mp::m / mp::s};
  frc::ExponentialProfile<mp::m, mp::V>::State state{0.0 * mp::m,
                                                     0.0 * mp::m / mp::s};

  auto prediction = profile.TimeLeftUntil(state, goal);
  auto reachedGoal = false;

  for (int i = 0; i < 900; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
    if (!reachedGoal && state == goal) {
      EXPECT_NEAR_UNITS(prediction, i * 10.0 * mp::ms, 250.0 * mp::ms);
      reachedGoal = true;
    }
  }

  EXPECT_EQ(state, goal);
}
