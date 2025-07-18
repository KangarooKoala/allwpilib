// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "frc/geometry/Rotation2d.h"
#include "frc/units.h"

using namespace frc;

TEST(Rotation2dTest, RadiansToDegrees) {
  const Rotation2d rot1{std::numbers::pi / 3.0 * mp::rad};
  const Rotation2d rot2{std::numbers::pi / 4.0 * mp::rad};

  EXPECT_DOUBLE_EQ(60.0, mp::value(rot1.Degrees()));
  EXPECT_DOUBLE_EQ(45.0, mp::value(rot2.Degrees()));
}

TEST(Rotation2dTest, DegreesToRadians) {
  const auto rot1 = Rotation2d{45.0 * mp::deg};
  const auto rot2 = Rotation2d{30.0 * mp::deg};

  EXPECT_DOUBLE_EQ(std::numbers::pi / 4.0, mp::value(rot1.Radians()));
  EXPECT_DOUBLE_EQ(std::numbers::pi / 6.0, mp::value(rot2.Radians()));
}

TEST(Rotation2dTest, RotateByFromZero) {
  const Rotation2d zero;
  auto rotated = zero + Rotation2d{90.0 * mp::deg};

  EXPECT_DOUBLE_EQ(std::numbers::pi / 2.0, mp::value(rotated.Radians()));
  EXPECT_DOUBLE_EQ(90.0, mp::value(rotated.Degrees()));
}

TEST(Rotation2dTest, RotateByNonZero) {
  auto rot = Rotation2d{90.0 * mp::deg};
  rot = rot + Rotation2d{30.0 * mp::deg};

  EXPECT_DOUBLE_EQ(120.0, mp::value(rot.Degrees()));
}

TEST(Rotation2dTest, Minus) {
  const auto rot1 = Rotation2d{70.0 * mp::deg};
  const auto rot2 = Rotation2d{30.0 * mp::deg};

  EXPECT_DOUBLE_EQ(40.0, mp::value((rot1 - rot2).Degrees()));
}

TEST(Rotation2dTest, UnaryMinus) {
  const auto rot = Rotation2d{20.0 * mp::deg};

  EXPECT_DOUBLE_EQ(-20.0, mp::value((-rot).Degrees()));
}

TEST(Rotation2dTest, Multiply) {
  const auto rot = Rotation2d{10.0 * mp::deg};

  EXPECT_DOUBLE_EQ(30.0, mp::value((rot * 3.0).Degrees()));
  EXPECT_DOUBLE_EQ(50.0, mp::value((rot * 41.0).Degrees()));
}

TEST(Rotation2dTest, Equality) {
  auto rot1 = Rotation2d{43.0 * mp::deg};
  auto rot2 = Rotation2d{43.0 * mp::deg};
  EXPECT_EQ(rot1, rot2);

  rot1 = Rotation2d{-180.0 * mp::deg};
  rot2 = Rotation2d{180.0 * mp::deg};
  EXPECT_EQ(rot1, rot2);
}

TEST(Rotation2dTest, Inequality) {
  const auto rot1 = Rotation2d{43.0 * mp::deg};
  const auto rot2 = Rotation2d{43.5 * mp::deg};
  EXPECT_NE(rot1, rot2);
}

TEST(Rotation2dTest, ToMatrix) {
#if __GNUC__ <= 11
  Rotation2d before{20.0 * mp::deg};
  Rotation2d after{before.ToMatrix()};
#else
  constexpr Rotation2d before{20.0 * mp::deg};
  constexpr Rotation2d after{before.ToMatrix()};
#endif

  EXPECT_EQ(before, after);
}

constexpr bool IsNear(mp::quantity<mp::rad> lhs, mp::quantity<mp::rad> rhs) {
  return lhs >= rhs - 1e-12 * mp::rad && lhs <= rhs + 1e-12 * mp::rad;
}

TEST(Rotation2dTest, Constexpr) {
  constexpr Rotation2d defaultCtor;
  constexpr Rotation2d radianCtor{5.0 * mp::rad};
  constexpr Rotation2d degreeCtor{270.0 * mp::deg};
  constexpr Rotation2d rotation45{45.0 * mp::deg};
  constexpr Rotation2d cartesianCtor{3.5, -3.5};

  constexpr auto negated = -radianCtor;
  constexpr auto multiplied = radianCtor * 2;
  constexpr auto subtracted = cartesianCtor - degreeCtor;

  static_assert(defaultCtor.Radians() == 0.0 * mp::rad);
  static_assert(degreeCtor.Degrees() == -90.0 * mp::deg);
  static_assert(IsNear(negated.Radians(), -5.0 * mp::rad + 1.0 * mp::rev));
  static_assert(IsNear(multiplied.Radians(), 10.0 * mp::rad - 2.0 * mp::rev));
  static_assert(subtracted == rotation45);
  static_assert(radianCtor != degreeCtor);
}
