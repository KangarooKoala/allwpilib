// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include <numbers>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <wpi/MathExtras.h>

#include "frc/geometry/Rotation3d.h"
#include "frc/units.h"

using namespace frc;

TEST(Rotation3dTest, GimbalLockAccuracy) {
  auto rot1 = Rotation3d{0.0 * mp::rad, 0.0 * mp::rad,
                         std::numbers::pi / 2.0 * mp::rad};
  auto rot2 =
      Rotation3d{std::numbers::pi * mp::rad, 0.0 * mp::rad, 0.0 * mp::rad};
  auto rot3 = Rotation3d{-std::numbers::pi / 2.0 * mp::rad, 0.0 * mp::rad,
                         0.0 * mp::rad};
  const auto result1 = rot1 + rot2 + rot3;
  const auto expected1 =
      Rotation3d{0.0 * mp::rad, -std::numbers::pi / 2.0 * mp::rad,
                 std::numbers::pi / 2.0 * mp::rad};
  EXPECT_EQ(expected1, result1);
  EXPECT_DOUBLE_EQ(std::numbers::pi / 2, mp::value(result1.X() + result1.Z()));
  EXPECT_NEAR(-std::numbers::pi / 2, mp::value(result1.Y()), 1e-7);

  rot1 = Rotation3d{0.0 * mp::rad, 0.0 * mp::rad,
                    std::numbers::pi / 2.0 * mp::rad};
  rot2 = Rotation3d{-std::numbers::pi * mp::rad, 0.0 * mp::rad, 0.0 * mp::rad};
  rot3 = Rotation3d{std::numbers::pi / 2.0 * mp::rad, 0.0 * mp::rad,
                    0.0 * mp::rad};
  const auto result2 = rot1 + rot2 + rot3;
  const auto expected2 =
      Rotation3d{0.0 * mp::rad, std::numbers::pi / 2.0 * mp::rad,
                 std::numbers::pi / 2.0 * mp::rad};
  EXPECT_EQ(expected2, result2);
  EXPECT_DOUBLE_EQ(std::numbers::pi / 2, mp::value(result2.Z() - result2.X()));
  EXPECT_NEAR(std::numbers::pi / 2, mp::value(result2.Y()), 1e-7);

  rot1 = Rotation3d{0.0 * mp::rad, 0.0 * mp::rad,
                    std::numbers::pi / 2.0 * mp::rad};
  rot2 = Rotation3d{0.0 * mp::rad, std::numbers::pi / 3.0 * mp::rad,
                    0.0 * mp::rad};
  rot3 = Rotation3d{-std::numbers::pi / 2.0 * mp::rad, 0.0 * mp::rad,
                    0.0 * mp::rad};
  const auto result3 = rot1 + rot2 + rot3;
  const auto expected3 =
      Rotation3d{0.0 * mp::rad, std::numbers::pi / 2.0 * mp::rad,
                 std::numbers::pi / 6.0 * mp::rad};
  EXPECT_EQ(expected3, result3);
  EXPECT_DOUBLE_EQ(std::numbers::pi / 6, mp::value(result3.Z() - result3.X()));
  EXPECT_DOUBLE_EQ(std::numbers::pi / 2, mp::value(result3.Y()));
}

TEST(Rotation3dTest, InitAxisAngleAndRollPitchYaw) {
  const Eigen::Vector3d xAxis{1.0, 0.0, 0.0};
  const Rotation3d rot1{xAxis, std::numbers::pi / 3.0 * mp::rad};
  const Rotation3d rot2{std::numbers::pi / 3.0 * mp::rad, 0.0 * mp::rad,
                        0.0 * mp::rad};
  const Rotation3d rvec1{Eigen::Vector3d{xAxis * std::numbers::pi / 3}};
  EXPECT_EQ(rot1, rot2);
  EXPECT_EQ(rot1, rvec1);

  const Eigen::Vector3d yAxis{0.0, 1.0, 0.0};
  const Rotation3d rot3{yAxis, std::numbers::pi / 3.0 * mp::rad};
  const Rotation3d rot4{0.0 * mp::rad, std::numbers::pi / 3.0 * mp::rad,
                        0.0 * mp::rad};
  const Rotation3d rvec2{Eigen::Vector3d{yAxis * std::numbers::pi / 3}};
  EXPECT_EQ(rot3, rot4);
  EXPECT_EQ(rot3, rvec2);

  const Eigen::Vector3d zAxis{0.0, 0.0, 1.0};
  const Rotation3d rot5{zAxis, std::numbers::pi / 3.0 * mp::rad};
  const Rotation3d rot6{0.0 * mp::rad, 0.0 * mp::rad,
                        std::numbers::pi / 3.0 * mp::rad};
  const Rotation3d rvec3{Eigen::Vector3d{zAxis * std::numbers::pi / 3}};
  EXPECT_EQ(rot5, rot6);
  EXPECT_EQ(rot5, rvec3);
}

TEST(Rotation3dTest, InitRotationMatrix) {
  // No rotation
  const Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
  const Rotation3d rot1{R1};
  EXPECT_EQ(Rotation3d{}, rot1);

  // 90 degree CCW rotation around z-axis
  Eigen::Matrix3d R2;
  R2.block<3, 1>(0, 0) = Eigen::Vector3d{0.0, 1.0, 0.0};
  R2.block<3, 1>(0, 1) = Eigen::Vector3d{-1.0, 0.0, 0.0};
  R2.block<3, 1>(0, 2) = Eigen::Vector3d{0.0, 0.0, 1.0};
  const Rotation3d rot2{R2};
  const Rotation3d expected2{0.0 * mp::deg, 0.0 * mp::deg, 90.0 * mp::deg};
  EXPECT_EQ(expected2, rot2);

  // Matrix that isn't orthogonal
  const Eigen::Matrix3d R3{{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  EXPECT_THROW(Rotation3d{R3}, std::domain_error);

  // Matrix that's orthogonal but not special orthogonal
  const Eigen::Matrix3d R4 = Eigen::Matrix3d::Identity() * 2.0;
  EXPECT_THROW(Rotation3d{R4}, std::domain_error);
}

TEST(Rotation3dTest, InitTwoVector) {
  const Eigen::Vector3d xAxis{1.0, 0.0, 0.0};
  const Eigen::Vector3d yAxis{0.0, 1.0, 0.0};
  const Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  // 90 degree CW rotation around y-axis
  const Rotation3d rot1{xAxis, zAxis};
  const Rotation3d expected1{yAxis, -std::numbers::pi / 2.0 * mp::rad};
  EXPECT_EQ(expected1, rot1);

  // 45 degree CCW rotation around z-axis
  const Rotation3d rot2{xAxis, Eigen::Vector3d{1.0, 1.0, 0.0}};
  const Rotation3d expected2{zAxis, std::numbers::pi / 4.0 * mp::rad};
  EXPECT_EQ(expected2, rot2);

  // 0 degree rotation of x-axes
  const Rotation3d rot3{xAxis, xAxis};
  EXPECT_EQ(Rotation3d{}, rot3);

  // 0 degree rotation of y-axes
  const Rotation3d rot4{yAxis, yAxis};
  EXPECT_EQ(Rotation3d{}, rot4);

  // 0 degree rotation of z-axes
  const Rotation3d rot5{zAxis, zAxis};
  EXPECT_EQ(Rotation3d{}, rot5);

  // 180 degree rotation tests. For 180 degree rotations, any quaternion with an
  // orthogonal rotation axis is acceptable. The rotation axis and initial
  // vector are orthogonal if their dot product is zero.

  // 180 degree rotation of x-axes
  const Rotation3d rot6{xAxis, -xAxis};
  const auto q6 = rot6.GetQuaternion();
  EXPECT_EQ(0.0, q6.W());
  EXPECT_EQ(0.0, q6.X() * xAxis(0) + q6.Y() * xAxis(1) + q6.Z() * xAxis(2));

  // 180 degree rotation of y-axes
  const Rotation3d rot7{yAxis, -yAxis};
  const auto q7 = rot7.GetQuaternion();
  EXPECT_EQ(0.0, q7.W());
  EXPECT_EQ(0.0, q7.X() * yAxis(0) + q7.Y() * yAxis(1) + q7.Z() * yAxis(2));

  // 180 degree rotation of z-axes
  const Rotation3d rot8{zAxis, -zAxis};
  const auto q8 = rot8.GetQuaternion();
  EXPECT_EQ(0.0, q8.W());
  EXPECT_EQ(0.0, q8.X() * zAxis(0) + q8.Y() * zAxis(1) + q8.Z() * zAxis(2));
}

TEST(Rotation3dTest, RadiansToDegrees) {
  const Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  const Rotation3d rot1{zAxis, std::numbers::pi / 3.0 * mp::rad};
  EXPECT_DOUBLE_EQ(0.0, mp::value(rot1.X()));
  EXPECT_DOUBLE_EQ(0.0, mp::value(rot1.Y()));
  EXPECT_DOUBLE_EQ(mp::value((60.0 * mp::deg).in(mp::rad)),
                   mp::value(rot1.Z()));

  const Rotation3d rot2{zAxis, std::numbers::pi / 4.0 * mp::rad};
  EXPECT_DOUBLE_EQ(0.0, mp::value(rot2.X()));
  EXPECT_DOUBLE_EQ(0.0, mp::value(rot2.Y()));
  EXPECT_DOUBLE_EQ(mp::value((45.0 * mp::deg).in(mp::rad)),
                   mp::value(rot2.Z()));
}

TEST(Rotation3dTest, DegreesToRadians) {
  const Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  const auto rot1 = Rotation3d{zAxis, 45.0 * mp::deg};
  EXPECT_DOUBLE_EQ(0.0, mp::value(rot1.X()));
  EXPECT_DOUBLE_EQ(0.0, mp::value(rot1.Y()));
  EXPECT_DOUBLE_EQ(std::numbers::pi / 4.0, mp::value(rot1.Z()));

  const auto rot2 = Rotation3d{zAxis, 30.0 * mp::deg};
  EXPECT_DOUBLE_EQ(0.0, mp::value(rot2.X()));
  EXPECT_DOUBLE_EQ(0.0, mp::value(rot2.Y()));
  EXPECT_DOUBLE_EQ(std::numbers::pi / 6.0, mp::value(rot2.Z()));
}

TEST(Rotation3dTest, RotationLoop) {
  Rotation3d rot;

  rot = rot + Rotation3d{90.0 * mp::deg, 0.0 * mp::deg, 0.0 * mp::deg};
  Rotation3d expected{90.0 * mp::deg, 0.0 * mp::deg, 0.0 * mp::deg};
  EXPECT_EQ(expected, rot);

  rot = rot + Rotation3d{0.0 * mp::deg, 90.0 * mp::deg, 0.0 * mp::deg};
  expected =
      Rotation3d{{1.0 / std::sqrt(3), 1.0 / std::sqrt(3), -1.0 / std::sqrt(3)},
                 120.0 * mp::deg};
  EXPECT_EQ(expected, rot);

  rot = rot + Rotation3d{0.0 * mp::deg, 0.0 * mp::deg, 90.0 * mp::deg};
  expected = Rotation3d{0.0 * mp::deg, 90.0 * mp::deg, 0.0 * mp::deg};
  EXPECT_EQ(expected, rot);

  rot = rot + Rotation3d{0.0 * mp::deg, -90.0 * mp::deg, 0.0 * mp::deg};
  EXPECT_EQ(Rotation3d{}, rot);
}

TEST(Rotation3dTest, RotateByFromZeroX) {
  const Eigen::Vector3d xAxis{1.0, 0.0, 0.0};

  const Rotation3d zero;
  auto rotated = zero + Rotation3d{xAxis, 90.0 * mp::deg};

  Rotation3d expected{xAxis, 90.0 * mp::deg};
  EXPECT_EQ(expected, rotated);
}

TEST(Rotation3dTest, RotateByFromZeroY) {
  const Eigen::Vector3d yAxis{0.0, 1.0, 0.0};

  const Rotation3d zero;
  auto rotated = zero + Rotation3d{yAxis, 90.0 * mp::deg};

  Rotation3d expected{yAxis, 90.0 * mp::deg};
  EXPECT_EQ(expected, rotated);
}

TEST(Rotation3dTest, RotateByFromZeroZ) {
  const Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  const Rotation3d zero;
  auto rotated = zero + Rotation3d{zAxis, 90.0 * mp::deg};

  Rotation3d expected{zAxis, 90.0 * mp::deg};
  EXPECT_EQ(expected, rotated);
}

TEST(Rotation3dTest, RotateByNonZeroX) {
  const Eigen::Vector3d xAxis{1.0, 0.0, 0.0};

  auto rot = Rotation3d{xAxis, 90.0 * mp::deg};
  rot = rot + Rotation3d{xAxis, 30.0 * mp::deg};

  Rotation3d expected{xAxis, 120.0 * mp::deg};
  EXPECT_EQ(expected, rot);
}

TEST(Rotation3dTest, RotateByNonZeroY) {
  const Eigen::Vector3d yAxis{0.0, 1.0, 0.0};

  auto rot = Rotation3d{yAxis, 90.0 * mp::deg};
  rot = rot + Rotation3d{yAxis, 30.0 * mp::deg};

  Rotation3d expected{yAxis, 120.0 * mp::deg};
  EXPECT_EQ(expected, rot);
}

TEST(Rotation3dTest, RotateByNonZeroZ) {
  const Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  auto rot = Rotation3d{zAxis, 90.0 * mp::deg};
  rot = rot + Rotation3d{zAxis, 30.0 * mp::deg};

  Rotation3d expected{zAxis, 120.0 * mp::deg};
  EXPECT_EQ(expected, rot);
}

TEST(Rotation3dTest, Minus) {
  const Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  const auto rot1 = Rotation3d{zAxis, 70.0 * mp::deg};
  const auto rot2 = Rotation3d{zAxis, 30.0 * mp::deg};

  EXPECT_DOUBLE_EQ(40.0, mp::value((rot1 - rot2).Z().in(mp::deg)));
}

TEST(Rotation3dTest, AxisAngle) {
  const Eigen::Vector3d xAxis{1.0, 0.0, 0.0};
  const Eigen::Vector3d yAxis{0.0, 1.0, 0.0};
  const Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  Rotation3d rot1{xAxis, 90.0 * mp::deg};
  EXPECT_EQ(xAxis, rot1.Axis());
  EXPECT_DOUBLE_EQ(std::numbers::pi / 2.0, mp::value(rot1.Angle()));

  Rotation3d rot2{yAxis, 45.0 * mp::deg};
  EXPECT_EQ(yAxis, rot2.Axis());
  EXPECT_DOUBLE_EQ(std::numbers::pi / 4.0, mp::value(rot2.Angle()));

  Rotation3d rot3{zAxis, 60.0 * mp::deg};
  EXPECT_EQ(zAxis, rot3.Axis());
  EXPECT_DOUBLE_EQ(std::numbers::pi / 3.0, mp::value(rot3.Angle()));
}

TEST(Rotation3dTest, ToRotation2d) {
  Rotation3d rotation{20.0 * mp::deg, 30.0 * mp::deg, 40.0 * mp::deg};
  Rotation2d expected{40.0 * mp::deg};

  EXPECT_EQ(expected, rotation.ToRotation2d());
}

TEST(Rotation3dTest, Equality) {
  const Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  const auto rot1 = Rotation3d{zAxis, 43.0 * mp::deg};
  const auto rot2 = Rotation3d{zAxis, 43.0 * mp::deg};
  EXPECT_EQ(rot1, rot2);

  const auto rot3 = Rotation3d{zAxis, -180.0 * mp::deg};
  const auto rot4 = Rotation3d{zAxis, 180.0 * mp::deg};
  EXPECT_EQ(rot3, rot4);
}

TEST(Rotation3dTest, Inequality) {
  const Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  const auto rot1 = Rotation3d{zAxis, 43.0 * mp::deg};
  const auto rot2 = Rotation3d{zAxis, 43.5 * mp::deg};
  EXPECT_NE(rot1, rot2);
}

TEST(Rotation3dTest, ToMatrix) {
#if __GNUC__ <= 11
  Rotation3d before{10.0 * mp::deg, 20.0 * mp::deg, 30.0 * mp::deg};
  Rotation3d after{before.ToMatrix()};
#else
  constexpr Rotation3d before{10.0 * mp::deg, 20.0 * mp::deg, 30.0 * mp::deg};
  constexpr Rotation3d after{before.ToMatrix()};
#endif

  EXPECT_EQ(before, after);
}

TEST(Rotation3dTest, Interpolate) {
  const Eigen::Vector3d xAxis{1.0, 0.0, 0.0};
  const Eigen::Vector3d yAxis{0.0, 1.0, 0.0};
  const Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  // 50 + (70 - 50) * 0.5 = 60
  auto rot1 = Rotation3d{xAxis, 50.0 * mp::deg};
  auto rot2 = Rotation3d{xAxis, 70.0 * mp::deg};
  auto interpolated = wpi::Lerp(rot1, rot2, 0.5);
  EXPECT_DOUBLE_EQ(60.0, mp::value(interpolated.X().in(mp::deg)));
  EXPECT_DOUBLE_EQ(0.0, mp::value(interpolated.Y().in(mp::deg)));
  EXPECT_DOUBLE_EQ(0.0, mp::value(interpolated.Z().in(mp::deg)));

  // -160 minus half distance between 170 and -160 (15) = -175
  rot1 = Rotation3d{xAxis, 170.0 * mp::deg};
  rot2 = Rotation3d{xAxis, -160.0 * mp::deg};
  interpolated = wpi::Lerp(rot1, rot2, 0.5);
  EXPECT_DOUBLE_EQ(-175.0, mp::value(interpolated.X().in(mp::deg)));
  EXPECT_DOUBLE_EQ(0.0, mp::value(interpolated.Y().in(mp::deg)));
  EXPECT_DOUBLE_EQ(0.0, mp::value(interpolated.Z().in(mp::deg)));

  // 50 + (70 - 50) * 0.5 = 60
  rot1 = Rotation3d{yAxis, 50.0 * mp::deg};
  rot2 = Rotation3d{yAxis, 70.0 * mp::deg};
  interpolated = wpi::Lerp(rot1, rot2, 0.5);
  EXPECT_DOUBLE_EQ(0.0, mp::value(interpolated.X().in(mp::deg)));
  EXPECT_DOUBLE_EQ(60.0, mp::value(interpolated.Y().in(mp::deg)));
  EXPECT_DOUBLE_EQ(0.0, mp::value(interpolated.Z().in(mp::deg)));

  // -160 plus half distance between 170 and -160 (165) = 5
  rot1 = Rotation3d{yAxis, 170.0 * mp::deg};
  rot2 = Rotation3d{yAxis, -160.0 * mp::deg};
  interpolated = wpi::Lerp(rot1, rot2, 0.5);
  EXPECT_DOUBLE_EQ(180.0, mp::value(interpolated.X().in(mp::deg)));
  EXPECT_DOUBLE_EQ(-5.0, mp::value(interpolated.Y().in(mp::deg)));
  EXPECT_DOUBLE_EQ(180.0, mp::value(interpolated.Z().in(mp::deg)));

  // 50 + (70 - 50) * 0.5 = 60
  rot1 = Rotation3d{zAxis, 50.0 * mp::deg};
  rot2 = Rotation3d{zAxis, 70.0 * mp::deg};
  interpolated = wpi::Lerp(rot1, rot2, 0.5);
  EXPECT_DOUBLE_EQ(0.0, mp::value(interpolated.X().in(mp::deg)));
  EXPECT_DOUBLE_EQ(0.0, mp::value(interpolated.Y().in(mp::deg)));
  EXPECT_DOUBLE_EQ(60.0, mp::value(interpolated.Z().in(mp::deg)));

  // -160 minus half distance between 170 and -160 (15) = -175
  rot1 = Rotation3d{zAxis, 170.0 * mp::deg};
  rot2 = Rotation3d{zAxis, -160.0 * mp::deg};
  interpolated = wpi::Lerp(rot1, rot2, 0.5);
  EXPECT_DOUBLE_EQ(0.0, mp::value(interpolated.X().in(mp::deg)));
  EXPECT_DOUBLE_EQ(0.0, mp::value(interpolated.Y().in(mp::deg)));
  EXPECT_DOUBLE_EQ(-175.0, mp::value(interpolated.Z().in(mp::deg)));
}
