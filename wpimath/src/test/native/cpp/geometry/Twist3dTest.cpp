// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "frc/geometry/Pose3d.h"
#include "frc/units.h"

using namespace frc;

TEST(Twist3dTest, StraightX) {
  const Twist3d straight{5.0 * mp::m,   0.0 * mp::m,   0.0 * mp::m,
                         0.0 * mp::rad, 0.0 * mp::rad, 0.0 * mp::rad};
  const auto straightPose = Pose3d{}.Exp(straight);

  Pose3d expected{5.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m, Rotation3d{}};
  EXPECT_EQ(expected, straightPose);
}

TEST(Twist3dTest, StraightY) {
  const Twist3d straight{0.0 * mp::m,   5.0 * mp::m,   0.0 * mp::m,
                         0.0 * mp::rad, 0.0 * mp::rad, 0.0 * mp::rad};
  const auto straightPose = Pose3d{}.Exp(straight);

  Pose3d expected{0.0 * mp::m, 5.0 * mp::m, 0.0 * mp::m, Rotation3d{}};
  EXPECT_EQ(expected, straightPose);
}

TEST(Twist3dTest, StraightZ) {
  const Twist3d straight{0.0 * mp::m,   0.0 * mp::m,   5.0 * mp::m,
                         0.0 * mp::rad, 0.0 * mp::rad, 0.0 * mp::rad};
  const auto straightPose = Pose3d{}.Exp(straight);

  Pose3d expected{0.0 * mp::m, 0.0 * mp::m, 5.0 * mp::m, Rotation3d{}};
  EXPECT_EQ(expected, straightPose);
}

TEST(Twist3dTest, QuarterCircle) {
  Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  const Twist3d quarterCircle{5.0 / 2.0 * std::numbers::pi * mp::m,
                              0.0 * mp::m,
                              0.0 * mp::m,
                              0.0 * mp::rad,
                              0.0 * mp::rad,
                              std::numbers::pi / 2.0 * mp::rad};
  const auto quarterCirclePose = Pose3d{}.Exp(quarterCircle);

  Pose3d expected{5.0 * mp::m, 5.0 * mp::m, 0.0 * mp::m,
                  Rotation3d{zAxis, 90.0 * mp::deg}};
  EXPECT_EQ(expected, quarterCirclePose);
}

TEST(Twist3dTest, DiagonalNoDtheta) {
  const Twist3d diagonal{2.0 * mp::m,   2.0 * mp::m,   0.0 * mp::m,
                         0.0 * mp::rad, 0.0 * mp::rad, 0.0 * mp::rad};
  const auto diagonalPose = Pose3d{}.Exp(diagonal);

  Pose3d expected{2.0 * mp::m, 2.0 * mp::m, 0.0 * mp::m, Rotation3d{}};
  EXPECT_EQ(expected, diagonalPose);
}

TEST(Twist3dTest, Equality) {
  const Twist3d one{5.0 * mp::m,   1.0 * mp::m,   0.0 * mp::m,
                    0.0 * mp::rad, 0.0 * mp::rad, 3.0 * mp::rad};
  const Twist3d two{5.0 * mp::m,   1.0 * mp::m,   0.0 * mp::m,
                    0.0 * mp::rad, 0.0 * mp::rad, 3.0 * mp::rad};
  EXPECT_TRUE(one == two);
}

TEST(Twist3dTest, Inequality) {
  const Twist3d one{5.0 * mp::m,   1.0 * mp::m,   0.0 * mp::m,
                    0.0 * mp::rad, 0.0 * mp::rad, 3.0 * mp::rad};
  const Twist3d two{5.0 * mp::m,   1.2 * mp::m,   0.0 * mp::m,
                    0.0 * mp::rad, 0.0 * mp::rad, 3.0 * mp::rad};
  EXPECT_TRUE(one != two);
}

TEST(Twist3dTest, Pose3dLogX) {
  const Pose3d end{0.0 * mp::m, 5.0 * mp::m, 5.0 * mp::m,
                   Rotation3d{90.0 * mp::deg, 0.0 * mp::deg, 0.0 * mp::deg}};
  const Pose3d start;

  const auto twist = start.Log(end);

  Twist3d expected{0.0 * mp::m,   5.0 / 2.0 * std::numbers::pi * mp::m,
                   0.0 * mp::m,   90.0 * mp::deg,
                   0.0 * mp::deg, 0.0 * mp::deg};
  EXPECT_EQ(expected, twist);

  // Make sure computed twist gives back original end pose
  const auto reapplied = start.Exp(twist);
  EXPECT_EQ(end, reapplied);
}

TEST(Twist3dTest, Pose3dLogY) {
  const Pose3d end{5.0 * mp::m, 0.0 * mp::m, 5.0 * mp::m,
                   Rotation3d{0.0 * mp::deg, 90.0 * mp::deg, 0.0 * mp::deg}};
  const Pose3d start;

  const auto twist = start.Log(end);

  Twist3d expected{
      0.0 * mp::m,   0.0 * mp::m,    5.0 / 2.0 * std::numbers::pi * mp::m,
      0.0 * mp::deg, 90.0 * mp::deg, 0.0 * mp::deg};
  EXPECT_EQ(expected, twist);

  // Make sure computed twist gives back original end pose
  const auto reapplied = start.Exp(twist);
  EXPECT_EQ(end, reapplied);
}

TEST(Twist3dTest, Pose3dLogZ) {
  const Pose3d end{5.0 * mp::m, 5.0 * mp::m, 0.0 * mp::m,
                   Rotation3d{0.0 * mp::deg, 0.0 * mp::deg, 90.0 * mp::deg}};
  const Pose3d start;

  const auto twist = start.Log(end);

  Twist3d expected{5.0 / 2.0 * std::numbers::pi * mp::m,
                   0.0 * mp::m,
                   0.0 * mp::m,
                   0.0 * mp::deg,
                   0.0 * mp::deg,
                   90.0 * mp::deg};
  EXPECT_EQ(expected, twist);

  // Make sure computed twist gives back original end pose
  const auto reapplied = start.Exp(twist);
  EXPECT_EQ(end, reapplied);
}

TEST(Twist3dTest, Constexpr) {
  constexpr Twist3d defaultCtor;
  constexpr Twist3d componentCtor{1.0 * mp::m,   2.0 * mp::m,   3.0 * mp::m,
                                  4.0 * mp::rad, 5.0 * mp::rad, 6.0 * mp::rad};
  constexpr auto multiplied = componentCtor * 2;

  static_assert(defaultCtor.dx == 0.0 * mp::m);
  static_assert(componentCtor.dy == 2.0 * mp::m);
  static_assert(componentCtor.dz == 3.0 * mp::m);
  static_assert(multiplied.dx == 2.0 * mp::m);
  static_assert(multiplied.rx == 8.0 * mp::rad);
  static_assert(multiplied.ry == 10.0 * mp::rad);
  static_assert(multiplied.rz == 12.0 * mp::rad);
}
