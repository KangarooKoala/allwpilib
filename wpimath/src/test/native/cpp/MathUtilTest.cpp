// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <limits>
#include <numbers>

#include <gtest/gtest.h>

#include "frc/MathUtil.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Translation3d.h"
#include "frc/units.h"

#define EXPECT_UNITS_EQ(a, b) EXPECT_DOUBLE_EQ(mp::value(a), mp::value(b))

#define EXPECT_UNITS_NEAR(a, b, c) EXPECT_NEAR(mp::value(a), mp::value(b), c)

TEST(MathUtilTest, ApplyDeadbandUnityScale) {
  // < 0
  EXPECT_DOUBLE_EQ(-1.0, frc::ApplyDeadband(-1.0, 0.02));
  EXPECT_DOUBLE_EQ((-0.03 + 0.02) / (1.0 - 0.02),
                   frc::ApplyDeadband(-0.03, 0.02));
  EXPECT_DOUBLE_EQ(0.0, frc::ApplyDeadband(-0.02, 0.02));
  EXPECT_DOUBLE_EQ(0.0, frc::ApplyDeadband(-0.01, 0.02));

  // == 0
  EXPECT_DOUBLE_EQ(0.0, frc::ApplyDeadband(0.0, 0.02));

  // > 0
  EXPECT_DOUBLE_EQ(0.0, frc::ApplyDeadband(0.01, 0.02));
  EXPECT_DOUBLE_EQ(0.0, frc::ApplyDeadband(0.02, 0.02));
  EXPECT_DOUBLE_EQ((0.03 - 0.02) / (1.0 - 0.02),
                   frc::ApplyDeadband(0.03, 0.02));
  EXPECT_DOUBLE_EQ(1.0, frc::ApplyDeadband(1.0, 0.02));
}

TEST(MathUtilTest, ApplyDeadbandArbitraryScale) {
  // < 0
  EXPECT_DOUBLE_EQ(-2.5, frc::ApplyDeadband(-2.5, 0.02, 2.5));
  EXPECT_DOUBLE_EQ(0.0, frc::ApplyDeadband(-0.02, 0.02, 2.5));
  EXPECT_DOUBLE_EQ(0.0, frc::ApplyDeadband(-0.01, 0.02, 2.5));

  // == 0
  EXPECT_DOUBLE_EQ(0.0, frc::ApplyDeadband(0.0, 0.02, 2.5));

  // > 0
  EXPECT_DOUBLE_EQ(0.0, frc::ApplyDeadband(0.01, 0.02, 2.5));
  EXPECT_DOUBLE_EQ(0.0, frc::ApplyDeadband(0.02, 0.02, 2.5));
  EXPECT_DOUBLE_EQ(2.5, frc::ApplyDeadband(2.5, 0.02, 2.5));
}

TEST(MathUtilTest, ApplyDeadbandUnits) {
  // < 0
  EXPECT_DOUBLE_EQ(-20, mp::value(frc::ApplyDeadband<mp::quantity<mp::rad>>(
                            -20.0 * mp::rad, 1.0 * mp::rad, 20.0 * mp::rad)));
}

TEST(MathUtilTest, ApplyDeadbandLargeMaxMagnitude) {
  EXPECT_DOUBLE_EQ(
      80.0,
      frc::ApplyDeadband(100.0, 20.0, std::numeric_limits<double>::infinity()));
}

TEST(MathUtilTest, CopySignPow) {
  EXPECT_DOUBLE_EQ(0.5, frc::CopySignPow(0.5, 1.0));
  EXPECT_DOUBLE_EQ(-0.5, frc::CopySignPow(-0.5, 1.0));

  EXPECT_DOUBLE_EQ(0.5 * 0.5, frc::CopySignPow(0.5, 2.0));
  EXPECT_DOUBLE_EQ(-(0.5 * 0.5), frc::CopySignPow(-0.5, 2.0));

  EXPECT_DOUBLE_EQ(std::sqrt(0.5), frc::CopySignPow(0.5, 0.5));
  EXPECT_DOUBLE_EQ(-std::sqrt(0.5), frc::CopySignPow(-0.5, 0.5));

  EXPECT_DOUBLE_EQ(0.0, frc::CopySignPow(0.0, 2.0));
  EXPECT_DOUBLE_EQ(1.0, frc::CopySignPow(1.0, 2.0));
  EXPECT_DOUBLE_EQ(-1.0, frc::CopySignPow(-1.0, 2.0));

  EXPECT_DOUBLE_EQ(std::pow(0.8, 0.3), frc::CopySignPow(0.8, 0.3));
  EXPECT_DOUBLE_EQ(-std::pow(0.8, 0.3), frc::CopySignPow(-0.8, 0.3));
}

TEST(MathUtilTest, CopySignPowWithMaxMagnitude) {
  EXPECT_DOUBLE_EQ(5.0, frc::CopySignPow(5.0, 1.0, 10.0));
  EXPECT_DOUBLE_EQ(-5.0, frc::CopySignPow(-5.0, 1.0, 10.0));

  EXPECT_DOUBLE_EQ(0.5 * 0.5 * 10, frc::CopySignPow(5.0, 2.0, 10.0));
  EXPECT_DOUBLE_EQ(-0.5 * 0.5 * 10, frc::CopySignPow(-5.0, 2.0, 10.0));

  EXPECT_DOUBLE_EQ(std::sqrt(0.5) * 10, frc::CopySignPow(5.0, 0.5, 10.0));
  EXPECT_DOUBLE_EQ(-std::sqrt(0.5) * 10, frc::CopySignPow(-5.0, 0.5, 10.0));

  EXPECT_DOUBLE_EQ(0.0, frc::CopySignPow(0.0, 2.0, 5.0));
  EXPECT_DOUBLE_EQ(5.0, frc::CopySignPow(5.0, 2.0, 5.0));
  EXPECT_DOUBLE_EQ(-5.0, frc::CopySignPow(-5.0, 2.0, 5.0));

  EXPECT_DOUBLE_EQ(std::pow(0.8, 0.3) * 100,
                   frc::CopySignPow(80.0, 0.3, 100.0));
  EXPECT_DOUBLE_EQ(-std::pow(0.8, 0.3) * 100,
                   frc::CopySignPow(-80.0, 0.3, 100.0));
}

TEST(MathUtilTest, CopySignPowWithUnits) {
  EXPECT_DOUBLE_EQ(0, mp::value(frc::CopySignPow<mp::quantity<mp::m / mp::s>>(
                          0.0 * mp::m / mp::s, 2.0)));
  EXPECT_DOUBLE_EQ(1, mp::value(frc::CopySignPow<mp::quantity<mp::m / mp::s>>(
                          1.0 * mp::m / mp::s, 2.0)));
  EXPECT_DOUBLE_EQ(-1, mp::value(frc::CopySignPow<mp::quantity<mp::m / mp::s>>(
                           -1.0 * mp::m / mp::s, 2.0)));

  EXPECT_DOUBLE_EQ(0.5 * 0.5 * 10,
                   mp::value(frc::CopySignPow<mp::quantity<mp::m / mp::s>>(
                       5.0 * mp::m / mp::s, 2.0, 10.0 * mp::m / mp::s)));
  EXPECT_DOUBLE_EQ(-0.5 * 0.5 * 10,
                   mp::value(frc::CopySignPow<mp::quantity<mp::m / mp::s>>(
                       -5.0 * mp::m / mp::s, 2.0, 10.0 * mp::m / mp::s)));
}

TEST(MathUtilTest, InputModulus) {
  // These tests check error wrapping. That is, the result of wrapping the
  // result of an angle reference minus the measurement.

  // Test symmetric range
  EXPECT_DOUBLE_EQ(-20.0, frc::InputModulus(170.0 - (-170.0), -180.0, 180.0));
  EXPECT_DOUBLE_EQ(-20.0,
                   frc::InputModulus(170.0 + 360.0 - (-170.0), -180.0, 180.0));
  EXPECT_DOUBLE_EQ(-20.0,
                   frc::InputModulus(170.0 - (-170.0 + 360.0), -180.0, 180.0));
  EXPECT_DOUBLE_EQ(20.0, frc::InputModulus(-170.0 - 170.0, -180.0, 180.0));
  EXPECT_DOUBLE_EQ(20.0,
                   frc::InputModulus(-170.0 + 360.0 - 170.0, -180.0, 180.0));
  EXPECT_DOUBLE_EQ(20.0,
                   frc::InputModulus(-170.0 - (170.0 + 360.0), -180.0, 180.0));

  // Test range starting at zero
  EXPECT_DOUBLE_EQ(340.0, frc::InputModulus(170.0 - 190.0, 0.0, 360.0));
  EXPECT_DOUBLE_EQ(340.0, frc::InputModulus(170.0 + 360.0 - 190.0, 0.0, 360.0));
  EXPECT_DOUBLE_EQ(340.0,
                   frc::InputModulus(170.0 - (190.0 + 360.0), 0.0, 360.0));

  // Test asymmetric range that doesn't start at zero
  EXPECT_DOUBLE_EQ(-20.0, frc::InputModulus(170.0 - (-170.0), -170.0, 190.0));

  // Test range with both positive endpoints
  EXPECT_DOUBLE_EQ(2.0, frc::InputModulus(0.0, 1.0, 3.0));
  EXPECT_DOUBLE_EQ(3.0, frc::InputModulus(1.0, 1.0, 3.0));
  EXPECT_DOUBLE_EQ(2.0, frc::InputModulus(2.0, 1.0, 3.0));
  EXPECT_DOUBLE_EQ(3.0, frc::InputModulus(3.0, 1.0, 3.0));
  EXPECT_DOUBLE_EQ(2.0, frc::InputModulus(4.0, 1.0, 3.0));

  // Test all supported types
  EXPECT_DOUBLE_EQ(-20.0,
                   frc::InputModulus<double>(170.0 - (-170.0), -170.0, 190.0));
  EXPECT_EQ(-20, frc::InputModulus<int>(170 - (-170), -170, 190));
  EXPECT_EQ(-20.0 * mp::deg, frc::InputModulus<mp::quantity<mp::deg>>(
                                 170.0 * mp::deg - (-170.0 * mp::deg),
                                 -170.0 * mp::deg, 190.0 * mp::deg));
}

TEST(MathUtilTest, AngleModulus) {
  EXPECT_UNITS_NEAR(
      frc::AngleModulus(-2000.0 * std::numbers::pi / 180.0 * mp::rad),
      160.0 * std::numbers::pi / 180.0 * mp::rad, 1e-10);
  EXPECT_UNITS_NEAR(
      frc::AngleModulus(358.0 * std::numbers::pi / 180.0 * mp::rad),
      -2.0 * std::numbers::pi / 180.0 * mp::rad, 1e-10);
  EXPECT_UNITS_NEAR(frc::AngleModulus(2.0 * std::numbers::pi * mp::rad),
                    0.0 * mp::rad, 1e-10);

  EXPECT_UNITS_EQ(frc::AngleModulus(5 * std::numbers::pi * mp::rad),
                  std::numbers::pi * mp::rad);
  EXPECT_UNITS_EQ(frc::AngleModulus(-5 * std::numbers::pi * mp::rad),
                  std::numbers::pi * mp::rad);
  EXPECT_UNITS_EQ(frc::AngleModulus(std::numbers::pi / 2.0 * mp::rad),
                  std::numbers::pi / 2.0 * mp::rad);
  EXPECT_UNITS_EQ(frc::AngleModulus(-std::numbers::pi / 2.0 * mp::rad),
                  -std::numbers::pi / 2.0 * mp::rad);
}

TEST(MathUtilTest, IsNear) {
  // The answer is always 42
  // Positive integer checks
  EXPECT_TRUE(frc::IsNear(42, 42, 1));
  EXPECT_TRUE(frc::IsNear(42, 41, 2));
  EXPECT_TRUE(frc::IsNear(42, 43, 2));
  EXPECT_FALSE(frc::IsNear(42, 44, 1));

  // Negative integer checks
  EXPECT_TRUE(frc::IsNear(-42, -42, 1));
  EXPECT_TRUE(frc::IsNear(-42, -41, 2));
  EXPECT_TRUE(frc::IsNear(-42, -43, 2));
  EXPECT_FALSE(frc::IsNear(-42, -44, 1));

  // Mixed sign integer checks
  EXPECT_FALSE(frc::IsNear(-42, 42, 1));
  EXPECT_FALSE(frc::IsNear(-42, 41, 2));
  EXPECT_FALSE(frc::IsNear(-42, 43, 2));
  EXPECT_FALSE(frc::IsNear(42, -42, 1));
  EXPECT_FALSE(frc::IsNear(42, -41, 2));
  EXPECT_FALSE(frc::IsNear(42, -43, 2));

  // Floating point checks
  EXPECT_TRUE(frc::IsNear<double>(42, 41.5, 1));
  EXPECT_TRUE(frc::IsNear<double>(42, 42.5, 1));
  EXPECT_TRUE(frc::IsNear<double>(42, 41.5, 0.75));
  EXPECT_TRUE(frc::IsNear<double>(42, 42.5, 0.75));

  // Wraparound checks
  EXPECT_TRUE(frc::IsNear(0.0 * mp::deg, 356.0 * mp::deg, 5.0 * mp::deg,
                          0.0 * mp::deg, 360.0 * mp::deg));
  EXPECT_TRUE(frc::IsNear(0, -356, 5, 0, 360));
  EXPECT_TRUE(frc::IsNear(0, 4, 5, 0, 360));
  EXPECT_TRUE(frc::IsNear(0, -4, 5, 0, 360));
  EXPECT_TRUE(frc::IsNear(400, 41, 5, 0, 360));
  EXPECT_TRUE(frc::IsNear(400, -319, 5, 0, 360));
  EXPECT_TRUE(frc::IsNear(400, 401, 5, 0, 360));
  EXPECT_FALSE(frc::IsNear<double>(0, 356, 2.5, 0, 360));
  EXPECT_FALSE(frc::IsNear<double>(0, -356, 2.5, 0, 360));
  EXPECT_FALSE(frc::IsNear<double>(0, 4, 2.5, 0, 360));
  EXPECT_FALSE(frc::IsNear<double>(0, -4, 2.5, 0, 360));
  EXPECT_FALSE(frc::IsNear(400, 35, 5, 0, 360));
  EXPECT_FALSE(frc::IsNear(400, -315, 5, 0, 360));
  EXPECT_FALSE(frc::IsNear(400, 395, 5, 0, 360));
  EXPECT_FALSE(frc::IsNear(0.0 * mp::deg, -4.0 * mp::deg, 2.5 * mp::deg,
                           0.0 * mp::deg, 360.0 * mp::deg));
}

TEST(MathUtilTest, Translation2dSlewRateLimitUnchanged) {
  const frc::Translation2d translation1{0.0 * mp::m, 0.0 * mp::m};
  const frc::Translation2d translation2{2.0 * mp::m, 2.0 * mp::m};

  const frc::Translation2d result1 = frc::SlewRateLimit(
      translation1, translation2, 1.0 * mp::s, 50.0 * mp::m / mp::s);

  const frc::Translation2d expected1{2.0 * mp::m, 2.0 * mp::m};

  EXPECT_EQ(result1, expected1);
}

TEST(MathUtilTest, Translation2dSlewRateLimitChanged) {
  const frc::Translation2d translation3{1.0 * mp::m, 1.0 * mp::m};
  const frc::Translation2d translation4{3.0 * mp::m, 3.0 * mp::m};

  const frc::Translation2d result2 = frc::SlewRateLimit(
      translation3, translation4, 0.25 * mp::s, 2.0 * mp::m / mp::s);

  const frc::Translation2d expected2{
      (1.0 + 0.5 * (std::numbers::sqrt2 / 2.0)) * mp::m,
      (1.0 + 0.5 * (std::numbers::sqrt2 / 2.0)) * mp::m};

  EXPECT_EQ(result2, expected2);
}

TEST(MathUtilTest, Translation3dSlewRateLimitUnchanged) {
  const frc::Translation3d translation1{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m};
  const frc::Translation3d translation2{2.0 * mp::m, 2.0 * mp::m, 2.0 * mp::m};

  const frc::Translation3d result1 = frc::SlewRateLimit(
      translation1, translation2, 1.0 * mp::s, 50.0 * mp::m / mp::s);

  const frc::Translation3d expected1{2.0 * mp::m, 2.0 * mp::m, 2.0 * mp::m};

  EXPECT_EQ(result1, expected1);
}

TEST(MathUtilTest, Translation3dSlewRateLimitChanged) {
  const frc::Translation3d translation3{1.0 * mp::m, 1.0 * mp::m, 1.0 * mp::m};
  const frc::Translation3d translation4{3.0 * mp::m, 3.0 * mp::m, 3.0 * mp::m};

  const frc::Translation3d result2 = frc::SlewRateLimit(
      translation3, translation4, 0.25 * mp::s, 2.0 * mp::m / mp::s);

  const frc::Translation3d expected2{
      (1.0 + 0.5 * std::numbers::inv_sqrt3) * mp::m,
      (1.0 + 0.5 * std::numbers::inv_sqrt3) * mp::m,
      (1.0 + 0.5 * std::numbers::inv_sqrt3) * mp::m};

  EXPECT_EQ(result2, expected2);
}
