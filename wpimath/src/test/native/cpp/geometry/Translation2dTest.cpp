// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include <gtest/gtest.h>

#include "frc/geometry/Translation2d.h"
#include "frc/units.h"

using namespace frc;

TEST(Translation2dTest, Sum) {
  const Translation2d one{1.0 * mp::m, 3.0 * mp::m};
  const Translation2d two{2.0 * mp::m, 5.0 * mp::m};

  const auto sum = one + two;

  EXPECT_DOUBLE_EQ(3.0, mp::value(sum.X()));
  EXPECT_DOUBLE_EQ(8.0, mp::value(sum.Y()));
}

TEST(Translation2dTest, Difference) {
  const Translation2d one{1.0 * mp::m, 3.0 * mp::m};
  const Translation2d two{2.0 * mp::m, 5.0 * mp::m};

  const auto difference = one - two;

  EXPECT_DOUBLE_EQ(-1.0, mp::value(difference.X()));
  EXPECT_DOUBLE_EQ(-2.0, mp::value(difference.Y()));
}

TEST(Translation2dTest, RotateBy) {
  const Translation2d another{3.0 * mp::m, 0.0 * mp::m};
  const auto rotated = another.RotateBy(90.0 * mp::deg);

  EXPECT_NEAR(0.0, mp::value(rotated.X()), 1e-9);
  EXPECT_NEAR(3.0, mp::value(rotated.Y()), 1e-9);
}

TEST(Translation2dTest, RotateAround) {
  const Translation2d translation{2.0 * mp::m, 1.0 * mp::m};
  const Translation2d other{3.0 * mp::m, 2.0 * mp::m};
  const auto rotated = translation.RotateAround(other, 180.0 * mp::deg);

  EXPECT_NEAR(4.0, mp::value(rotated.X()), 1e-9);
  EXPECT_NEAR(3.0, mp::value(rotated.Y()), 1e-9);
}

TEST(Translation2dTest, Multiplication) {
  const Translation2d original{3.0 * mp::m, 5.0 * mp::m};
  const auto mult = original * 3;

  EXPECT_DOUBLE_EQ(9.0, mp::value(mult.X()));
  EXPECT_DOUBLE_EQ(15.0, mp::value(mult.Y()));
}

TEST(Translation2dTest, Division) {
  const Translation2d original{3.0 * mp::m, 5.0 * mp::m};
  const auto div = original / 2;

  EXPECT_DOUBLE_EQ(1.5, mp::value(div.X()));
  EXPECT_DOUBLE_EQ(2.5, mp::value(div.Y()));
}

TEST(Translation2dTest, Norm) {
  const Translation2d one{3.0 * mp::m, 5.0 * mp::m};
  EXPECT_DOUBLE_EQ(std::hypot(3.0, 5.0), mp::value(one.Norm()));
}

TEST(Translation2dTest, SquaredNorm) {
  const Translation2d one{3.0 * mp::m, 5.0 * mp::m};
  EXPECT_DOUBLE_EQ(34.0, mp::value(one.SquaredNorm()));
}

TEST(Translation2dTest, Distance) {
  const Translation2d one{1.0 * mp::m, 1.0 * mp::m};
  const Translation2d two{6.0 * mp::m, 6.0 * mp::m};
  EXPECT_DOUBLE_EQ(5.0 * std::sqrt(2.0), mp::value(one.Distance(two)));
}

TEST(Translation2dTest, SquaredDistance) {
  const Translation2d one{1.0 * mp::m, 1.0 * mp::m};
  const Translation2d two{6.0 * mp::m, 6.0 * mp::m};
  EXPECT_DOUBLE_EQ(50.0, mp::value(one.SquaredDistance(two)));
}

TEST(Translation2dTest, UnaryMinus) {
  const Translation2d original{-4.5 * mp::m, 7.0 * mp::m};
  const auto inverted = -original;

  EXPECT_DOUBLE_EQ(4.5, mp::value(inverted.X()));
  EXPECT_DOUBLE_EQ(-7.0, mp::value(inverted.Y()));
}

TEST(Translation2dTest, Equality) {
  const Translation2d one{9.0 * mp::m, 5.5 * mp::m};
  const Translation2d two{9.0 * mp::m, 5.5 * mp::m};
  EXPECT_TRUE(one == two);
}

TEST(Translation2dTest, Inequality) {
  const Translation2d one{9.0 * mp::m, 5.5 * mp::m};
  const Translation2d two{9.0 * mp::m, 5.7 * mp::m};
  EXPECT_TRUE(one != two);
}

TEST(Translation2dTest, PolarConstructor) {
  Translation2d one{std::sqrt(2) * mp::m, Rotation2d{45.0 * mp::deg}};
  EXPECT_DOUBLE_EQ(1.0, mp::value(one.X()));
  EXPECT_DOUBLE_EQ(1.0, mp::value(one.Y()));

  Translation2d two{2.0 * mp::m, Rotation2d{60.0 * mp::deg}};
  EXPECT_DOUBLE_EQ(1.0, mp::value(two.X()));
  EXPECT_DOUBLE_EQ(std::sqrt(3.0), mp::value(two.Y()));
}

TEST(Translation2dTest, Nearest) {
  const Translation2d origin{0.0 * mp::m, 0.0 * mp::m};

  const Translation2d translation1{1.0 * mp::m, Rotation2d{45.0 * mp::deg}};
  const Translation2d translation2{2.0 * mp::m, Rotation2d{90.0 * mp::deg}};
  const Translation2d translation3{3.0 * mp::m, Rotation2d{135.0 * mp::deg}};
  const Translation2d translation4{4.0 * mp::m, Rotation2d{180.0 * mp::deg}};
  const Translation2d translation5{5.0 * mp::m, Rotation2d{270.0 * mp::deg}};

  EXPECT_DOUBLE_EQ(
      mp::value(origin.Nearest({translation5, translation3, translation4}).X()),
      mp::value(translation3.X()));
  EXPECT_DOUBLE_EQ(
      mp::value(origin.Nearest({translation5, translation3, translation4}).Y()),
      mp::value(translation3.Y()));

  EXPECT_DOUBLE_EQ(
      mp::value(origin.Nearest({translation1, translation2, translation3}).X()),
      mp::value(translation1.X()));
  EXPECT_DOUBLE_EQ(
      mp::value(origin.Nearest({translation1, translation2, translation3}).Y()),
      mp::value(translation1.Y()));

  EXPECT_DOUBLE_EQ(
      mp::value(origin.Nearest({translation4, translation2, translation3}).X()),
      mp::value(translation2.X()));
  EXPECT_DOUBLE_EQ(
      mp::value(origin.Nearest({translation4, translation2, translation3}).Y()),
      mp::value(translation2.Y()));
}

TEST(Translation2dTest, ToVector) {
  const Eigen::Vector2d vec(1.0, 2.0);
  const Translation2d translation{vec};

  EXPECT_DOUBLE_EQ(vec[0], mp::value(translation.X()));
  EXPECT_DOUBLE_EQ(vec[1], mp::value(translation.Y()));

  EXPECT_TRUE(vec == translation.ToVector());
}

TEST(Translation2dTest, Constexpr) {
  constexpr Translation2d defaultCtor;
  constexpr Translation2d componentCtor{1.0 * mp::m, 2.0 * mp::m};
  constexpr auto added = defaultCtor + componentCtor;
  constexpr auto subtracted = defaultCtor - componentCtor;
  constexpr auto negated = -componentCtor;
  constexpr auto multiplied = componentCtor * 2;
  constexpr auto divided = componentCtor / 2;

  static_assert(defaultCtor.X() == 0.0 * mp::m);
  static_assert(componentCtor.Y() == 2.0 * mp::m);
  static_assert(added.X() == 1.0 * mp::m);
  static_assert(subtracted.Y() == (-2.0 * mp::m));
  static_assert(negated.X() == (-1.0 * mp::m));
  static_assert(multiplied.X() == 2.0 * mp::m);
  static_assert(divided.Y() == 1.0 * mp::m);
}

TEST(Translation2dTest, Dot) {
  const Translation2d one{2.0 * mp::m, 3.0 * mp::m};
  const Translation2d two{3.0 * mp::m, 4.0 * mp::m};
  EXPECT_DOUBLE_EQ(18.0, mp::value(one.Dot(two)));
}

TEST(Translation2dTest, Cross) {
  const Translation2d one{2.0 * mp::m, 3.0 * mp::m};
  const Translation2d two{3.0 * mp::m, 4.0 * mp::m};
  EXPECT_DOUBLE_EQ(-1.0, mp::value(one.Cross(two)));
}
