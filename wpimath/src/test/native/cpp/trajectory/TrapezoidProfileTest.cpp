// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/trajectory/TrapezoidProfile.h"  // NOLINT(build/include_order)

#include <chrono>
#include <cmath>

#include <gtest/gtest.h>

#include "frc/units.h"

static constexpr auto kDt = 10.0 * mp::ms;

#define EXPECT_NEAR_UNITS(val1, val2, eps) EXPECT_LE(mp::abs(val1 - val2), eps)

#define EXPECT_LT_OR_NEAR_UNITS(val1, val2, eps) \
  if (val1 <= val2) {                            \
    EXPECT_LE(val1, val2);                       \
  } else {                                       \
    EXPECT_NEAR_UNITS(val1, val2, eps);          \
  }

TEST(TrapezoidProfileTest, ReachesGoal) {
  frc::TrapezoidProfile<mp::m>::Constraints constraints{1.75 * mp::m / mp::s,
                                                        0.75 * mp::m / mp::s2};
  frc::TrapezoidProfile<mp::m>::State goal{3.0 * mp::m, 0.0 * mp::m / mp::s};
  frc::TrapezoidProfile<mp::m>::State state;

  frc::TrapezoidProfile<mp::m> profile{constraints};
  for (int i = 0; i < 450; ++i) {
    state = profile.Calculate(kDt, state, goal);
  }
  EXPECT_EQ(state, goal);
}

// Tests that decreasing the maximum velocity in the middle when it is already
// moving faster than the new max is handled correctly
TEST(TrapezoidProfileTest, PosContinuousUnderVelChange) {
  frc::TrapezoidProfile<mp::m>::Constraints constraints{1.75 * mp::m / mp::s,
                                                        0.75 * mp::m / mp::s2};
  frc::TrapezoidProfile<mp::m>::State goal{12.0 * mp::m, 0.0 * mp::m / mp::s};

  frc::TrapezoidProfile<mp::m> profile{constraints};
  auto state =
      profile.Calculate(kDt, frc::TrapezoidProfile<mp::m>::State{}, goal);

  auto lastPos = state.position;
  for (int i = 0; i < 1600; ++i) {
    if (i == 400) {
      constraints.maxVelocity = 0.75 * mp::m / mp::s;
      profile = frc::TrapezoidProfile<mp::m>{constraints};
    }

    state = profile.Calculate(kDt, state, goal);
    auto estimatedVel = (state.position - lastPos) / kDt;

    if (i >= 400) {
      // Since estimatedVel can have floating point rounding errors, we check
      // whether value is less than or within an error delta of the new
      // constraint.
      EXPECT_LT_OR_NEAR_UNITS(estimatedVel, constraints.maxVelocity,
                              1e-4 * mp::m / mp::s);

      EXPECT_LE(state.velocity, constraints.maxVelocity);
    }

    lastPos = state.position;
  }
  EXPECT_EQ(state, goal);
}

// There is some somewhat tricky code for dealing with going backwards
TEST(TrapezoidProfileTest, Backwards) {
  frc::TrapezoidProfile<mp::m>::Constraints constraints{0.75 * mp::m / mp::s,
                                                        0.75 * mp::m / mp::s2};
  frc::TrapezoidProfile<mp::m>::State goal{-2.0 * mp::m, 0.0 * mp::m / mp::s};
  frc::TrapezoidProfile<mp::m>::State state;

  frc::TrapezoidProfile<mp::m> profile{constraints};
  for (int i = 0; i < 400; ++i) {
    state = profile.Calculate(kDt, state, goal);
  }
  EXPECT_EQ(state, goal);
}

TEST(TrapezoidProfileTest, SwitchGoalInMiddle) {
  frc::TrapezoidProfile<mp::m>::Constraints constraints{0.75 * mp::m / mp::s,
                                                        0.75 * mp::m / mp::s2};
  frc::TrapezoidProfile<mp::m>::State goal{-2.0 * mp::m, 0.0 * mp::m / mp::s};
  frc::TrapezoidProfile<mp::m>::State state;

  frc::TrapezoidProfile<mp::m> profile{constraints};
  for (int i = 0; i < 200; ++i) {
    state = profile.Calculate(kDt, state, goal);
  }
  EXPECT_NE(state, goal);

  goal = {0.0 * mp::m, 0.0 * mp::m / mp::s};
  profile = frc::TrapezoidProfile<mp::m>{constraints};
  for (int i = 0; i < 550; ++i) {
    state = profile.Calculate(kDt, state, goal);
  }
  EXPECT_EQ(state, goal);
}

// Checks to make sure that it hits top speed
TEST(TrapezoidProfileTest, TopSpeed) {
  frc::TrapezoidProfile<mp::m>::Constraints constraints{0.75 * mp::m / mp::s,
                                                        0.75 * mp::m / mp::s2};
  frc::TrapezoidProfile<mp::m>::State goal{4.0 * mp::m, 0.0 * mp::m / mp::s};
  frc::TrapezoidProfile<mp::m>::State state;

  frc::TrapezoidProfile<mp::m> profile{constraints};
  for (int i = 0; i < 200; ++i) {
    state = profile.Calculate(kDt, state, goal);
  }
  EXPECT_NEAR_UNITS(constraints.maxVelocity, state.velocity,
                    1e-4 * mp::m / mp::s);

  profile = frc::TrapezoidProfile<mp::m>{constraints};
  for (int i = 0; i < 2000; ++i) {
    state = profile.Calculate(kDt, state, goal);
  }
  EXPECT_EQ(state, goal);
}

TEST(TrapezoidProfileTest, TimingToCurrent) {
  frc::TrapezoidProfile<mp::m>::Constraints constraints{0.75 * mp::m / mp::s,
                                                        0.75 * mp::m / mp::s2};
  frc::TrapezoidProfile<mp::m>::State goal{2.0 * mp::m, 0.0 * mp::m / mp::s};
  frc::TrapezoidProfile<mp::m>::State state;

  frc::TrapezoidProfile<mp::m> profile{constraints};
  for (int i = 0; i < 400; i++) {
    state = profile.Calculate(kDt, state, goal);
    EXPECT_NEAR_UNITS(profile.TimeLeftUntil(state.position), 0.0 * mp::s,
                      2e-2 * mp::s);
  }
}

TEST(TrapezoidProfileTest, TimingToGoal) {
  frc::TrapezoidProfile<mp::m>::Constraints constraints{0.75 * mp::m / mp::s,
                                                        0.75 * mp::m / mp::s2};
  frc::TrapezoidProfile<mp::m>::State goal{2.0 * mp::m, 0.0 * mp::m / mp::s};

  frc::TrapezoidProfile<mp::m> profile{constraints};
  auto state =
      profile.Calculate(kDt, goal, frc::TrapezoidProfile<mp::m>::State{});

  auto predictedTimeLeft = profile.TimeLeftUntil(goal.position);
  bool reachedGoal = false;
  for (int i = 0; i < 400; i++) {
    state = profile.Calculate(kDt, state, goal);
    if (!reachedGoal && state == goal) {
      // Expected value using for loop index is just an approximation since the
      // time left in the profile doesn't increase linearly at the endpoints
      EXPECT_NEAR(mp::value(predictedTimeLeft), i / 100.0, 0.25);
      reachedGoal = true;
    }
  }
}

TEST(TrapezoidProfileTest, TimingBeforeGoal) {
  frc::TrapezoidProfile<mp::m>::Constraints constraints{0.75 * mp::m / mp::s,
                                                        0.75 * mp::m / mp::s2};
  frc::TrapezoidProfile<mp::m>::State goal{2.0 * mp::m, 0.0 * mp::m / mp::s};

  frc::TrapezoidProfile<mp::m> profile{constraints};
  auto state =
      profile.Calculate(kDt, goal, frc::TrapezoidProfile<mp::m>::State{});

  auto predictedTimeLeft = profile.TimeLeftUntil(1.0 * mp::m);
  bool reachedGoal = false;
  for (int i = 0; i < 400; i++) {
    state = profile.Calculate(kDt, state, goal);
    if (!reachedGoal && (mp::abs(state.velocity - 1.0 * mp::m / mp::s) <
                         10e-5 * mp::m / mp::s)) {
      EXPECT_NEAR(mp::value(predictedTimeLeft), i / 100.0, 2e-2);
      reachedGoal = true;
    }
  }
}

TEST(TrapezoidProfileTest, TimingToNegativeGoal) {
  frc::TrapezoidProfile<mp::m>::Constraints constraints{0.75 * mp::m / mp::s,
                                                        0.75 * mp::m / mp::s2};
  frc::TrapezoidProfile<mp::m>::State goal{-2.0 * mp::m, 0.0 * mp::m / mp::s};

  frc::TrapezoidProfile<mp::m> profile{constraints};
  auto state =
      profile.Calculate(kDt, goal, frc::TrapezoidProfile<mp::m>::State{});

  auto predictedTimeLeft = profile.TimeLeftUntil(goal.position);
  bool reachedGoal = false;
  for (int i = 0; i < 400; i++) {
    state = profile.Calculate(kDt, state, goal);
    if (!reachedGoal && state == goal) {
      // Expected value using for loop index is just an approximation since the
      // time left in the profile doesn't increase linearly at the endpoints
      EXPECT_NEAR(mp::value(predictedTimeLeft), i / 100.0, 0.25);
      reachedGoal = true;
    }
  }
}

TEST(TrapezoidProfileTest, TimingBeforeNegativeGoal) {
  frc::TrapezoidProfile<mp::m>::Constraints constraints{0.75 * mp::m / mp::s,
                                                        0.75 * mp::m / mp::s2};
  frc::TrapezoidProfile<mp::m>::State goal{-2.0 * mp::m, 0.0 * mp::m / mp::s};

  frc::TrapezoidProfile<mp::m> profile{constraints};
  auto state =
      profile.Calculate(kDt, goal, frc::TrapezoidProfile<mp::m>::State{});

  auto predictedTimeLeft = profile.TimeLeftUntil(-1.0 * mp::m);
  bool reachedGoal = false;
  for (int i = 0; i < 400; i++) {
    state = profile.Calculate(kDt, state, goal);
    if (!reachedGoal && (mp::abs(state.velocity + 1.0 * mp::m / mp::s) <
                         10e-5 * mp::m / mp::s)) {
      EXPECT_NEAR(mp::value(predictedTimeLeft), i / 100.0, 2e-2);
      reachedGoal = true;
    }
  }
}

TEST(TrapezoidProfileTest, InitalizationOfCurrentState) {
  frc::TrapezoidProfile<mp::m>::Constraints constraints{1.0 * mp::m / mp::s,
                                                        1.0 * mp::m / mp::s2};
  frc::TrapezoidProfile<mp::m> profile{constraints};
  EXPECT_NEAR_UNITS(profile.TimeLeftUntil(0.0 * mp::m), 0.0 * mp::s,
                    1e-10 * mp::s);
  EXPECT_NEAR_UNITS(profile.TotalTime(), 0.0 * mp::s, 1e-10 * mp::s);
}

TEST(TrapezoidProfileTest, InitialVelocityConstraints) {
  frc::TrapezoidProfile<mp::m>::Constraints constraints{0.75 * mp::m / mp::s,
                                                               0.75 * mp::m / mp::s2};
  frc::TrapezoidProfile<mp::m>::State goal{10.0 * mp::m, 0.0 * mp::m / mp::s};
  frc::TrapezoidProfile<mp::m>::State state{0.0 * mp::m, -10.0 * mp::m / mp::s};

  frc::TrapezoidProfile<mp::m> profile{constraints};

  for (int i = 0; i < 200; ++i) {
    state = profile.Calculate(kDt, state, goal);
    EXPECT_LE(mp::abs(state.velocity),
              mp::abs(constraints.maxVelocity));
  }
}

TEST(TrapezoidProfileTest, GoalVelocityConstraints) {
  frc::TrapezoidProfile<mp::m>::Constraints constraints{0.75 * mp::m / mp::s,
                                                               0.75 * mp::m / mp::s2};
  frc::TrapezoidProfile<mp::m>::State goal{10.0 * mp::m, 5.0 * mp::m / mp::s};
  frc::TrapezoidProfile<mp::m>::State state{0.0 * mp::m, 0.75 * mp::m / mp::s};

  frc::TrapezoidProfile<mp::m> profile{constraints};

  for (int i = 0; i < 200; ++i) {
    state = profile.Calculate(kDt, state, goal);
    EXPECT_LE(mp::abs(state.velocity),
              mp::abs(constraints.maxVelocity));
  }
}

TEST(TrapezoidProfileTest, NegativeGoalVelocityConstraints) {
  frc::TrapezoidProfile<mp::m>::Constraints constraints{0.75 * mp::m / mp::s,
                                                               0.75 * mp::m / mp::s2};
  frc::TrapezoidProfile<mp::m>::State goal{10.0 * mp::m, -5.0 * mp::m / mp::s};
  frc::TrapezoidProfile<mp::m>::State state{0.0 * mp::m, 0.75 * mp::m / mp::s};

  frc::TrapezoidProfile<mp::m> profile{constraints};

  for (int i = 0; i < 200; ++i) {
    state = profile.Calculate(kDt, state, goal);
    EXPECT_LE(mp::abs(state.velocity),
              mp::abs(constraints.maxVelocity));
  }
}
