// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/math/trajectory/ExponentialProfile.hpp"

#include <chrono>
#include <cmath>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>
#include <wpi/units/acceleration.h>
#include <wpi/units/frequency.h>
#include <wpi/units/length.h>
#include <wpi/units/velocity.h>
#include <wpi/units/voltage.h>

#include "wpi/math/controller/SimpleMotorFeedforward.hpp"

static constexpr auto kDt = 10_ms;
static constexpr auto kV = 2.5629_V / 1_mps;
static constexpr auto kA = 0.43277_V / 1_mps2;

#define EXPECT_NEAR_UNITS(val1, val2, eps) \
  EXPECT_LE(wpi::units::abs(val1 - val2), eps)

#define EXPECT_LT_OR_NEAR_UNITS(val1, val2, eps) \
  if (val1 <= val2) {                            \
    EXPECT_LE(val1, val2);                       \
  } else {                                       \
    EXPECT_NEAR_UNITS(val1, val2, eps);          \
  }

wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
CheckDynamics(
    wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>
        profile,
    wpi::math::ExponentialProfile<wpi::units::meters_,
                                  wpi::units::volts_>::Constraints constraints,
    wpi::math::SimpleMotorFeedforward<wpi::units::meters_> feedforward,
    wpi::math::ExponentialProfile<wpi::units::meters_,
                                  wpi::units::volts_>::State current,
    wpi::math::ExponentialProfile<wpi::units::meters_,
                                  wpi::units::volts_>::State goal) {
  auto next = profile.Calculate(kDt, current, goal);
  auto signal = feedforward.Calculate(current.velocity, next.velocity);

  EXPECT_LE(wpi::units::abs(signal), (constraints.maxInput + 1e-9_V));

  return next;
}

TEST(ExponentialProfileTest, ReachesGoal) {
  wpi::math::ExponentialProfile<wpi::units::meters_,
                                wpi::units::volts_>::Constraints constraints{
      12_V, -kV / kA, 1 / kA};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>
      profile{constraints};
  wpi::math::SimpleMotorFeedforward<wpi::units::meters_> feedforward{
      0_V, 2.5629_V / 1_mps, 0.43277_V / 1_mps2, kDt};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      goal{10_m, 0_mps};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      state{0_m, 0_mps};

  for (int i = 0; i < 450; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
  }
  EXPECT_EQ(state, goal);
}

// Tests that decreasing the maximum velocity in the middle when it is already
// moving faster than the new max is handled correctly
TEST(ExponentialProfileTest, PosContinuousUnderVelChange) {
  wpi::math::ExponentialProfile<wpi::units::meters_,
                                wpi::units::volts_>::Constraints constraints{
      12_V, -kV / kA, 1 / kA};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>
      profile{constraints};
  wpi::math::SimpleMotorFeedforward<wpi::units::meters_> feedforward{
      0_V, 2.5629_V / 1_mps, 0.43277_V / 1_mps2, kDt};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      goal{10_m, 0_mps};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      state{0_m, 0_mps};

  for (int i = 0; i < 300; ++i) {
    if (i == 150) {
      constraints.maxInput = 9_V;
      profile = wpi::math::ExponentialProfile<wpi::units::meters_,
                                              wpi::units::volts_>{constraints};
    }

    state = CheckDynamics(profile, constraints, feedforward, state, goal);
  }
  EXPECT_EQ(state, goal);
}

// Tests that decreasing the maximum velocity in the middle when it is already
// moving faster than the new max is handled correctly
TEST(ExponentialProfileTest, PosContinuousUnderVelChangeBackward) {
  wpi::math::ExponentialProfile<wpi::units::meters_,
                                wpi::units::volts_>::Constraints constraints{
      12_V, -kV / kA, 1 / kA};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>
      profile{constraints};
  wpi::math::SimpleMotorFeedforward<wpi::units::meters_> feedforward{
      0_V, 2.5629_V / 1_mps, 0.43277_V / 1_mps2, kDt};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      goal{-10_m, 0_mps};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      state{0_m, 0_mps};

  for (int i = 0; i < 300; ++i) {
    if (i == 150) {
      constraints.maxInput = 9_V;
      profile = wpi::math::ExponentialProfile<wpi::units::meters_,
                                              wpi::units::volts_>{constraints};
    }

    state = CheckDynamics(profile, constraints, feedforward, state, goal);
  }
  EXPECT_EQ(state, goal);
}

// There is some somewhat tricky code for dealing with going backwards
TEST(ExponentialProfileTest, Backwards) {
  wpi::math::ExponentialProfile<wpi::units::meters_,
                                wpi::units::volts_>::Constraints constraints{
      12_V, -kV / kA, 1 / kA};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>
      profile{constraints};
  wpi::math::SimpleMotorFeedforward<wpi::units::meters_> feedforward{
      0_V, 2.5629_V / 1_mps, 0.43277_V / 1_mps2, kDt};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      goal{-10_m, 0_mps};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      state;

  for (int i = 0; i < 400; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
  }
  EXPECT_EQ(state, goal);
}

TEST(ExponentialProfileTest, SwitchGoalInMiddle) {
  wpi::math::ExponentialProfile<wpi::units::meters_,
                                wpi::units::volts_>::Constraints constraints{
      12_V, -kV / kA, 1 / kA};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>
      profile{constraints};
  wpi::math::SimpleMotorFeedforward<wpi::units::meters_> feedforward{
      0_V, 2.5629_V / 1_mps, 0.43277_V / 1_mps2, kDt};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      goal{-10_m, 0_mps};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      state{0_m, 0_mps};

  for (int i = 0; i < 50; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
  }
  EXPECT_NE(state, goal);

  goal = {0.0_m, 0.0_mps};
  for (int i = 0; i < 100; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
  }
  EXPECT_EQ(state, goal);
}

// Checks to make sure that it hits top speed on long trajectories
TEST(ExponentialProfileTest, TopSpeed) {
  wpi::math::ExponentialProfile<wpi::units::meters_,
                                wpi::units::volts_>::Constraints constraints{
      12_V, -kV / kA, 1 / kA};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>
      profile{constraints};
  wpi::math::SimpleMotorFeedforward<wpi::units::meters_> feedforward{
      0_V, 2.5629_V / 1_mps, 0.43277_V / 1_mps2, kDt};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      goal{40_m, 0_mps};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      state;

  wpi::units::meters_per_second<> maxSpeed = 0_mps;

  for (int i = 0; i < 900; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
    maxSpeed = wpi::units::max(state.velocity, maxSpeed);
  }

  EXPECT_NEAR_UNITS(constraints.MaxVelocity(), maxSpeed, 1e-5_mps);
  EXPECT_EQ(state, goal);
}

// Checks to make sure that it hits top speed on long trajectories
TEST(ExponentialProfileTest, TopSpeedBackward) {
  wpi::math::ExponentialProfile<wpi::units::meters_,
                                wpi::units::volts_>::Constraints constraints{
      12_V, -kV / kA, 1 / kA};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>
      profile{constraints};
  wpi::math::SimpleMotorFeedforward<wpi::units::meters_> feedforward{
      0_V, 2.5629_V / 1_mps, 0.43277_V / 1_mps2, kDt};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      goal{-40_m, 0_mps};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      state;

  wpi::units::meters_per_second<> maxSpeed = 0_mps;

  for (int i = 0; i < 900; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
    maxSpeed = wpi::units::min(state.velocity, maxSpeed);
  }

  EXPECT_NEAR_UNITS(-constraints.MaxVelocity(), maxSpeed, 1e-5_mps);
  EXPECT_EQ(state, goal);
}

// Checks to make sure that it hits top speed on long trajectories
TEST(ExponentialProfileTest, HighInitialSpeed) {
  wpi::math::ExponentialProfile<wpi::units::meters_,
                                wpi::units::volts_>::Constraints constraints{
      12_V, -kV / kA, 1 / kA};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>
      profile{constraints};
  wpi::math::SimpleMotorFeedforward<wpi::units::meters_> feedforward{
      0_V, 2.5629_V / 1_mps, 0.43277_V / 1_mps2, kDt};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      goal{40_m, 0_mps};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      state{0_m, 8_mps};

  for (int i = 0; i < 900; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
  }

  EXPECT_EQ(state, goal);
}

// Checks to make sure that it hits top speed on long trajectories
TEST(ExponentialProfileTest, HighInitialSpeedBackward) {
  wpi::math::ExponentialProfile<wpi::units::meters_,
                                wpi::units::volts_>::Constraints constraints{
      12_V, -kV / kA, 1 / kA};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>
      profile{constraints};
  wpi::math::SimpleMotorFeedforward<wpi::units::meters_> feedforward{
      0_V, 2.5629_V / 1_mps, 0.43277_V / 1_mps2, kDt};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      goal{-40_m, 0_mps};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      state{0_m, -8_mps};

  for (int i = 0; i < 900; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
  }

  EXPECT_EQ(state, goal);
}

TEST(ExponentialProfileTest, TestHeuristic) {
  wpi::math::ExponentialProfile<wpi::units::meters_,
                                wpi::units::volts_>::Constraints constraints{
      12_V, -kV / kA, 1 / kA};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>
      profile{constraints};
  std::vector<std::tuple<
      wpi::math::ExponentialProfile<wpi::units::meters_,
                                    wpi::units::volts_>::State,  // initial
      wpi::math::ExponentialProfile<wpi::units::meters_,
                                    wpi::units::volts_>::State,  // goal
      wpi::math::ExponentialProfile<wpi::units::meters_,
                                    wpi::units::volts_>::State>  // inflection
                                                                 // point
              >
      testCases{
          // red > green and purple => always positive => false
          {{0_m, -4_mps}, {0.75_m, -4_mps}, {1.3758_m, 4.4304_mps}},
          {{0_m, -4_mps}, {1.4103_m, 4_mps}, {1.3758_m, 4.4304_mps}},
          {{0.6603_m, 4_mps}, {0.75_m, -4_mps}, {1.3758_m, 4.4304_mps}},
          {{0.6603_m, 4_mps}, {1.4103_m, 4_mps}, {1.3758_m, 4.4304_mps}},

          // purple > red > green => positive if v0 < 0 => c && !d && a
          {{0_m, -4_mps}, {0.5_m, -2_mps}, {0.4367_m, 3.7217_mps}},
          {{0_m, -4_mps}, {0.546_m, 2_mps}, {0.4367_m, 3.7217_mps}},
          {{0.6603_m, 4_mps}, {0.5_m, -2_mps}, {0.5560_m, -2.9616_mps}},
          {{0.6603_m, 4_mps}, {0.546_m, 2_mps}, {0.5560_m, -2.9616_mps}},

          // red < green and purple => always negative => true => !c && !d
          {{0_m, -4_mps}, {-0.75_m, -4_mps}, {-0.7156_m, -4.4304_mps}},
          {{0_m, -4_mps}, {-0.0897_m, 4_mps}, {-0.7156_m, -4.4304_mps}},
          {{0.6603_m, 4_mps}, {-0.75_m, -4_mps}, {-0.7156_m, -4.4304_mps}},
          {{0.6603_m, 4_mps}, {-0.0897_m, 4_mps}, {-0.7156_m, -4.4304_mps}},

          // green > red > purple => positive if vf < 0 => !c && d && b
          {{0_m, -4_mps}, {-0.5_m, -4.5_mps}, {1.095_m, 4.314_mps}},
          {{0_m, -4_mps}, {1.0795_m, 4.5_mps}, {-0.5122_m, -4.351_mps}},
          {{0.6603_m, 4_mps}, {-0.5_m, -4.5_mps}, {1.095_m, 4.314_mps}},
          {{0.6603_m, 4_mps}, {1.0795_m, 4.5_mps}, {-0.5122_m, -4.351_mps}},

          // tests for initial velocity > V/kV
          {{0_m, -8_mps}, {0_m, 0_mps}, {-0.1384_m, 3.342_mps}},
          {{0_m, -8_mps}, {-1_m, 0_mps}, {-0.562_m, -6.792_mps}},
          {{0_m, 8_mps}, {1_m, 0_mps}, {0.562_m, 6.792_mps}},
          {{0_m, 8_mps}, {-1_m, 0_mps}, {-0.785_m, -4.346_mps}},
      };

  for (auto& testCase : testCases) {
    auto state = profile.CalculateInflectionPoint(std::get<0>(testCase),
                                                  std::get<1>(testCase));
    EXPECT_NEAR_UNITS(std::get<2>(testCase).position / 1_m,
                      state.position / 1_m, 1e-3);
    EXPECT_NEAR_UNITS(std::get<2>(testCase).velocity / 1_mps,
                      state.velocity / 1_mps, 1e-3);
  }
}

TEST(ExponentialProfileTest, TimingToCurrent) {
  wpi::math::ExponentialProfile<wpi::units::meters_,
                                wpi::units::volts_>::Constraints constraints{
      12_V, -kV / kA, 1 / kA};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>
      profile{constraints};
  wpi::math::SimpleMotorFeedforward<wpi::units::meters_> feedforward{
      0_V, 2.5629_V / 1_mps, 0.43277_V / 1_mps2, kDt};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      goal{2_m, 0_mps};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      state{0_m, 0_mps};

  for (int i = 0; i < 900; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
    EXPECT_NEAR_UNITS(profile.TimeLeftUntil(state, state), 0_s, 2e-2_s);
  }

  EXPECT_EQ(state, goal);
}

TEST(ExponentialProfileTest, TimingToGoal) {
  wpi::math::ExponentialProfile<wpi::units::meters_,
                                wpi::units::volts_>::Constraints constraints{
      12_V, -kV / kA, 1 / kA};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>
      profile{constraints};
  wpi::math::SimpleMotorFeedforward<wpi::units::meters_> feedforward{
      0_V, 2.5629_V / 1_mps, 0.43277_V / 1_mps2, kDt};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      goal{2_m, 0_mps};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      state{0_m, 0_mps};

  auto prediction = profile.TimeLeftUntil(state, goal);
  auto reachedGoal = false;

  for (int i = 0; i < 900; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
    if (!reachedGoal && state == goal) {
      EXPECT_NEAR_UNITS(prediction, i * 10_ms, 250_ms);
      reachedGoal = true;
    }
  }

  EXPECT_EQ(state, goal);
}

TEST(ExponentialProfileTest, TimingToNegativeGoal) {
  wpi::math::ExponentialProfile<wpi::units::meters_,
                                wpi::units::volts_>::Constraints constraints{
      12_V, -kV / kA, 1 / kA};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>
      profile{constraints};
  wpi::math::SimpleMotorFeedforward<wpi::units::meters_> feedforward{
      0_V, 2.5629_V / 1_mps, 0.43277_V / 1_mps2, kDt};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      goal{-2_m, 0_mps};
  wpi::math::ExponentialProfile<wpi::units::meters_, wpi::units::volts_>::State
      state{0_m, 0_mps};

  auto prediction = profile.TimeLeftUntil(state, goal);
  auto reachedGoal = false;

  for (int i = 0; i < 900; ++i) {
    state = CheckDynamics(profile, constraints, feedforward, state, goal);
    if (!reachedGoal && state == goal) {
      EXPECT_NEAR_UNITS(prediction, i * 10_ms, 250_ms);
      reachedGoal = true;
    }
  }

  EXPECT_EQ(state, goal);
}
