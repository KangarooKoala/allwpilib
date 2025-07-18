// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include <gtest/gtest.h>

#include "frc/geometry/Translation3d.h"
#include "frc/units.h"

using namespace frc;

static constexpr double kEpsilon = 1E-9;

TEST(Translation3dTest, Sum) {
  const Translation3d one{1.0 * mp::m, 3.0 * mp::m, 5.0 * mp::m};
  const Translation3d two{2.0 * mp::m, 5.0 * mp::m, 8.0 * mp::m};

  const auto sum = one + two;

  EXPECT_NEAR(3.0, mp::value(sum.X()), kEpsilon);
  EXPECT_NEAR(8.0, mp::value(sum.Y()), kEpsilon);
  EXPECT_NEAR(13.0, mp::value(sum.Z()), kEpsilon);
}

TEST(Translation3dTest, Difference) {
  const Translation3d one{1.0 * mp::m, 3.0 * mp::m, 5.0 * mp::m};
  const Translation3d two{2.0 * mp::m, 5.0 * mp::m, 8.0 * mp::m};

  const auto difference = one - two;

  EXPECT_NEAR(mp::value(difference.X()), -1.0, kEpsilon);
  EXPECT_NEAR(mp::value(difference.Y()), -2.0, kEpsilon);
  EXPECT_NEAR(mp::value(difference.Z()), -3.0, kEpsilon);
}

TEST(Translation3dTest, RotateBy) {
  Eigen::Vector3d xAxis{1.0, 0.0, 0.0};
  Eigen::Vector3d yAxis{0.0, 1.0, 0.0};
  Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  const Translation3d translation{1.0 * mp::m, 2.0 * mp::m, 3.0 * mp::m};

  const auto rotated1 = translation.RotateBy(Rotation3d{xAxis, 90.0 * mp::deg});
  EXPECT_NEAR(mp::value(rotated1.X()), 1.0, kEpsilon);
  EXPECT_NEAR(mp::value(rotated1.Y()), -3.0, kEpsilon);
  EXPECT_NEAR(mp::value(rotated1.Z()), 2.0, kEpsilon);

  const auto rotated2 = translation.RotateBy(Rotation3d{yAxis, 90.0 * mp::deg});
  EXPECT_NEAR(mp::value(rotated2.X()), 3.0, kEpsilon);
  EXPECT_NEAR(mp::value(rotated2.Y()), 2.0, kEpsilon);
  EXPECT_NEAR(mp::value(rotated2.Z()), -1.0, kEpsilon);

  const auto rotated3 = translation.RotateBy(Rotation3d{zAxis, 90.0 * mp::deg});
  EXPECT_NEAR(mp::value(rotated3.X()), -2.0, kEpsilon);
  EXPECT_NEAR(mp::value(rotated3.Y()), 1.0, kEpsilon);
  EXPECT_NEAR(mp::value(rotated3.Z()), 3.0, kEpsilon);
}

TEST(Translation3dTest, RotateAround) {
  Eigen::Vector3d xAxis{1.0, 0.0, 0.0};
  Eigen::Vector3d yAxis{0.0, 1.0, 0.0};
  Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  const Translation3d translation{1.0 * mp::m, 2.0 * mp::m, 3.0 * mp::m};
  const Translation3d around{3.0 * mp::m, 2.0 * mp::m, 1.0 * mp::m};

  const auto rotated1 =
      translation.RotateAround(around, Rotation3d{xAxis, 90.0 * mp::deg});
  EXPECT_NEAR(mp::value(rotated1.X()), 1.0, kEpsilon);
  EXPECT_NEAR(mp::value(rotated1.Y()), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(rotated1.Z()), 1.0, kEpsilon);

  const auto rotated2 =
      translation.RotateAround(around, Rotation3d{yAxis, 90.0 * mp::deg});
  EXPECT_NEAR(mp::value(rotated2.X()), 5.0, kEpsilon);
  EXPECT_NEAR(mp::value(rotated2.Y()), 2.0, kEpsilon);
  EXPECT_NEAR(mp::value(rotated2.Z()), 3.0, kEpsilon);

  const auto rotated3 =
      translation.RotateAround(around, Rotation3d{zAxis, 90.0 * mp::deg});
  EXPECT_NEAR(mp::value(rotated3.X()), 3.0, kEpsilon);
  EXPECT_NEAR(mp::value(rotated3.Y()), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(rotated3.Z()), 3.0, kEpsilon);
}

TEST(Translation3dTest, ToTranslation2d) {
  Translation3d translation{1.0 * mp::m, 2.0 * mp::m, 3.0 * mp::m};
  Translation2d expected{1.0 * mp::m, 2.0 * mp::m};

  EXPECT_EQ(expected, translation.ToTranslation2d());
}

TEST(Translation3dTest, Multiplication) {
  const Translation3d original{3.0 * mp::m, 5.0 * mp::m, 7.0 * mp::m};
  const auto mult = original * 3;

  EXPECT_NEAR(mp::value(mult.X()), 9.0, kEpsilon);
  EXPECT_NEAR(mp::value(mult.Y()), 15.0, kEpsilon);
  EXPECT_NEAR(mp::value(mult.Z()), 21.0, kEpsilon);
}

TEST(Translation3dTest, Division) {
  const Translation3d original{3.0 * mp::m, 5.0 * mp::m, 7.0 * mp::m};
  const auto div = original / 2;

  EXPECT_NEAR(mp::value(div.X()), 1.5, kEpsilon);
  EXPECT_NEAR(mp::value(div.Y()), 2.5, kEpsilon);
  EXPECT_NEAR(mp::value(div.Z()), 3.5, kEpsilon);
}

TEST(Translation3dTest, Norm) {
  const Translation3d one{3.0 * mp::m, 5.0 * mp::m, 7.0 * mp::m};
  EXPECT_NEAR(mp::value(one.Norm()), std::hypot(3, 5, 7), kEpsilon);
}

TEST(Translation3dTest, SquaredNorm) {
  const Translation3d one{3.0 * mp::m, 5.0 * mp::m, 7.0 * mp::m};
  EXPECT_NEAR(mp::value(one.SquaredNorm()), 83.0, kEpsilon);
}

TEST(Translation3dTest, Distance) {
  const Translation3d one{1.0 * mp::m, 1.0 * mp::m, 1.0 * mp::m};
  const Translation3d two{6.0 * mp::m, 6.0 * mp::m, 6.0 * mp::m};
  EXPECT_NEAR(mp::value(one.Distance(two)), 5 * std::sqrt(3), kEpsilon);
}

TEST(Translation3dTest, SquaredDistance) {
  const Translation3d one{1.0 * mp::m, 1.0 * mp::m, 1.0 * mp::m};
  const Translation3d two{6.0 * mp::m, 6.0 * mp::m, 6.0 * mp::m};
  EXPECT_NEAR(mp::value(one.SquaredDistance(two)), 75.0, kEpsilon);
}

TEST(Translation3dTest, UnaryMinus) {
  const Translation3d original{-4.5 * mp::m, 7.0 * mp::m, 9.0 * mp::m};
  const auto inverted = -original;

  EXPECT_NEAR(mp::value(inverted.X()), 4.5, kEpsilon);
  EXPECT_NEAR(mp::value(inverted.Y()), -7, kEpsilon);
  EXPECT_NEAR(mp::value(inverted.Z()), -9, kEpsilon);
}

TEST(Translation3dTest, Equality) {
  const Translation3d one{9.0 * mp::m, 5.5 * mp::m, 3.5 * mp::m};
  const Translation3d two{9.0 * mp::m, 5.5 * mp::m, 3.5 * mp::m};
  EXPECT_TRUE(one == two);
}

TEST(Translation3dTest, Inequality) {
  const Translation3d one{9.0 * mp::m, 5.5 * mp::m, 3.5 * mp::m};
  const Translation3d two{9.0 * mp::m, 5.7 * mp::m, 3.5 * mp::m};
  EXPECT_TRUE(one != two);
}

TEST(Translation3dTest, PolarConstructor) {
  Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  Translation3d one{std::sqrt(2) * 1.0 * mp::m,
                    Rotation3d{zAxis, 45.0 * mp::deg}};
  EXPECT_NEAR(mp::value(one.X()), 1.0, kEpsilon);
  EXPECT_NEAR(mp::value(one.Y()), 1.0, kEpsilon);
  EXPECT_NEAR(mp::value(one.Z()), 0.0, kEpsilon);

  Translation3d two{2.0 * mp::m, Rotation3d{zAxis, 60.0 * mp::deg}};
  EXPECT_NEAR(mp::value(two.X()), 1.0, kEpsilon);
  EXPECT_NEAR(mp::value(two.Y()), std::sqrt(3.0), kEpsilon);
  EXPECT_NEAR(mp::value(two.Z()), 0.0, kEpsilon);
}

TEST(Translation3dTest, ToVector) {
  const Eigen::Vector3d vec(1.0, 2.0, 3.0);
  const Translation3d translation{vec};

  EXPECT_DOUBLE_EQ(vec[0], mp::value(translation.X()));
  EXPECT_DOUBLE_EQ(vec[1], mp::value(translation.Y()));
  EXPECT_DOUBLE_EQ(vec[2], mp::value(translation.Z()));

  EXPECT_TRUE(vec == translation.ToVector());
}

TEST(Translation3dTest, Constexpr) {
  constexpr Translation3d defaultCtor;
  constexpr Translation3d componentCtor{1.0 * mp::m, 2.0 * mp::m, 3.0 * mp::m};
  constexpr auto added = defaultCtor + componentCtor;
  constexpr auto subtracted = defaultCtor - componentCtor;
  constexpr auto negated = -componentCtor;
  constexpr auto multiplied = componentCtor * 2;
  constexpr auto divided = componentCtor / 2;
  constexpr Translation2d projected = componentCtor.ToTranslation2d();

  static_assert(defaultCtor.X() == 0.0 * mp::m);
  static_assert(componentCtor.Y() == 2.0 * mp::m);
  static_assert(added.Z() == 3.0 * mp::m);
  static_assert(subtracted.X() == (-1.0 * mp::m));
  static_assert(negated.Y() == (-2.0 * mp::m));
  static_assert(multiplied.Z() == 6.0 * mp::m);
  static_assert(divided.Y() == 1.0 * mp::m);
  static_assert(projected.X() == 1.0 * mp::m);
  static_assert(projected.Y() == 2.0 * mp::m);
}

TEST(Translation3dTest, Nearest) {
  const Translation3d origin{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m};

  // Distance sort
  // translations are in order of closest to farthest away from the origin at
  // various positions in 3D space.
  const Translation3d translation1{1.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m};
  const Translation3d translation2{0.0 * mp::m, 2.0 * mp::m, 0.0 * mp::m};
  const Translation3d translation3{0.0 * mp::m, 0.0 * mp::m, 3.0 * mp::m};
  const Translation3d translation4{2.0 * mp::m, 2.0 * mp::m, 2.0 * mp::m};
  const Translation3d translation5{3.0 * mp::m, 3.0 * mp::m, 3.0 * mp::m};

  auto nearest1 = origin.Nearest({translation5, translation3, translation4});
  EXPECT_DOUBLE_EQ(mp::value(nearest1.X()), mp::value(translation3.X()));
  EXPECT_DOUBLE_EQ(mp::value(nearest1.Y()), mp::value(translation3.Y()));
  EXPECT_DOUBLE_EQ(mp::value(nearest1.Z()), mp::value(translation3.Z()));

  auto nearest2 = origin.Nearest({translation1, translation2, translation3});
  EXPECT_DOUBLE_EQ(mp::value(nearest2.X()), mp::value(translation1.X()));
  EXPECT_DOUBLE_EQ(mp::value(nearest2.Y()), mp::value(translation1.Y()));
  EXPECT_DOUBLE_EQ(mp::value(nearest2.Z()), mp::value(translation1.Z()));

  auto nearest3 = origin.Nearest({translation4, translation2, translation3});
  EXPECT_DOUBLE_EQ(mp::value(nearest3.X()), mp::value(translation2.X()));
  EXPECT_DOUBLE_EQ(mp::value(nearest3.Y()), mp::value(translation2.Y()));
  EXPECT_DOUBLE_EQ(mp::value(nearest3.Z()), mp::value(translation2.Z()));
}

TEST(Translation3dTest, Dot) {
  const Translation3d one{1.0 * mp::m, 2.0 * mp::m, 3.0 * mp::m};
  const Translation3d two{4.0 * mp::m, 5.0 * mp::m, 6.0 * mp::m};
  EXPECT_NEAR(mp::value(one.Dot(two)), 32.0, kEpsilon);
}

TEST(Translation3dTest, Cross) {
  const Translation3d one{1.0 * mp::m, 2.0 * mp::m, 3.0 * mp::m};
  const Translation3d two{4.0 * mp::m, 5.0 * mp::m, 6.0 * mp::m};

  auto cross = one.Cross(two);
  EXPECT_NEAR(mp::value(cross[0]), -3.0, kEpsilon);
  EXPECT_NEAR(mp::value(cross[1]), 6.0, kEpsilon);
  EXPECT_NEAR(mp::value(cross[2]), -3.0, kEpsilon);
}
