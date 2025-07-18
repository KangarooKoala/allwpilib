// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "frc/geometry/Pose2d.h"
#include "frc/units.h"

using namespace frc;

TEST(Twist2dTest, Straight) {
  const Twist2d straight{5.0 * mp::m, 0.0 * mp::m, 0.0 * mp::rad};
  const auto straightPose = straight.Exp();

  EXPECT_DOUBLE_EQ(5.0, mp::value(straightPose.X()));
  EXPECT_DOUBLE_EQ(0.0, mp::value(straightPose.Y()));
  EXPECT_DOUBLE_EQ(0.0, mp::value(straightPose.Rotation().Radians()));
}

TEST(Twist2dTest, QuarterCircle) {
  const Twist2d quarterCircle{5.0 * mp::m / 2.0 * std::numbers::pi, 0.0 * mp::m,
                              std::numbers::pi / 2.0 * mp::rad};
  const auto quarterCirclePose = quarterCircle.Exp();

  EXPECT_DOUBLE_EQ(5.0, mp::value(quarterCirclePose.X()));
  EXPECT_DOUBLE_EQ(5.0, mp::value(quarterCirclePose.Y()));
  EXPECT_DOUBLE_EQ(90.0, mp::value(quarterCirclePose.Rotation().Degrees()));
}

TEST(Twist2dTest, DiagonalNoDtheta) {
  const Twist2d diagonal{2.0 * mp::m, 2.0 * mp::m, 0.0 * mp::deg};
  const auto diagonalPose = diagonal.Exp();

  EXPECT_DOUBLE_EQ(2.0, mp::value(diagonalPose.X()));
  EXPECT_DOUBLE_EQ(2.0, mp::value(diagonalPose.Y()));
  EXPECT_DOUBLE_EQ(0.0, mp::value(diagonalPose.Rotation().Degrees()));
}

TEST(Twist2dTest, Equality) {
  const Twist2d one{5.0 * mp::m, 1.0 * mp::m, 3.0 * mp::rad};
  const Twist2d two{5.0 * mp::m, 1.0 * mp::m, 3.0 * mp::rad};
  EXPECT_TRUE(one == two);
}

TEST(Twist2dTest, Inequality) {
  const Twist2d one{5.0 * mp::m, 1.0 * mp::m, 3.0 * mp::rad};
  const Twist2d two{5.0 * mp::m, 1.2 * mp::m, 3.0 * mp::rad};
  EXPECT_TRUE(one != two);
}

TEST(Twist2dTest, Pose2dLog) {
  const Pose2d end{5.0 * mp::m, 5.0 * mp::m, 90.0 * mp::deg};
  const Pose2d start;

  const auto twist = (end - start).Log();

  Twist2d expected{5.0 / 2.0 * std::numbers::pi * mp::m, 0.0 * mp::m,
                   std::numbers::pi / 2.0 * mp::rad};
  EXPECT_EQ(expected, twist);

  // Make sure computed twist gives back original end pose
  const auto reapplied = start + twist.Exp();
  EXPECT_EQ(end, reapplied);
}

TEST(Twist2dTest, Constexpr) {
  constexpr Twist2d defaultCtor;
  constexpr Twist2d componentCtor{1.0 * mp::m, 2.0 * mp::m, 3.0 * mp::rad};
  constexpr auto multiplied = componentCtor * 2;

  static_assert(defaultCtor.dx == 0.0 * mp::m);
  static_assert(componentCtor.dy == 2.0 * mp::m);
  static_assert(multiplied.dtheta == 6.0 * mp::rad);
}
