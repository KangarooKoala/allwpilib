// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/kinematics/MecanumDriveWheelSpeeds.h"
#include "frc/units.h"

TEST(MecanumDriveWheelSpeedsTest, Plus) {
  const frc::MecanumDriveWheelSpeeds left{
      1.0 * mp::m / mp::s, 0.5 * mp::m / mp::s, 2.0 * mp::m / mp::s,
      1.5 * mp::m / mp::s};
  const frc::MecanumDriveWheelSpeeds right{
      2.0 * mp::m / mp::s, 1.5 * mp::m / mp::s, 0.5 * mp::m / mp::s,
      1.0 * mp::m / mp::s};

  const frc::MecanumDriveWheelSpeeds result = left + right;

  EXPECT_EQ(3.0, mp::value(result.frontLeft));
  EXPECT_EQ(2.0, mp::value(result.frontRight));
  EXPECT_EQ(2.5, mp::value(result.rearLeft));
  EXPECT_EQ(2.5, mp::value(result.rearRight));
}

TEST(MecanumDriveWheelSpeedsTest, Minus) {
  const frc::MecanumDriveWheelSpeeds left{
      1.0 * mp::m / mp::s, 0.5 * mp::m / mp::s, 2.0 * mp::m / mp::s,
      1.5 * mp::m / mp::s};
  const frc::MecanumDriveWheelSpeeds right{
      2.0 * mp::m / mp::s, 1.5 * mp::m / mp::s, 0.5 * mp::m / mp::s,
      1.0 * mp::m / mp::s};

  const frc::MecanumDriveWheelSpeeds result = left - right;

  EXPECT_EQ(-1.0, mp::value(result.frontLeft));
  EXPECT_EQ(-1.0, mp::value(result.frontRight));
  EXPECT_EQ(1.5, mp::value(result.rearLeft));
  EXPECT_EQ(0.5, mp::value(result.rearRight));
}

TEST(MecanumDriveWheelSpeedsTest, UnaryMinus) {
  const frc::MecanumDriveWheelSpeeds speeds{
      1.0 * mp::m / mp::s, 0.5 * mp::m / mp::s, 2.0 * mp::m / mp::s,
      1.5 * mp::m / mp::s};

  const frc::MecanumDriveWheelSpeeds result = -speeds;

  EXPECT_EQ(-1.0, mp::value(result.frontLeft));
  EXPECT_EQ(-0.5, mp::value(result.frontRight));
  EXPECT_EQ(-2.0, mp::value(result.rearLeft));
  EXPECT_EQ(-1.5, mp::value(result.rearRight));
}

TEST(MecanumDriveWheelSpeedsTest, Multiplication) {
  const frc::MecanumDriveWheelSpeeds speeds{
      1.0 * mp::m / mp::s, 0.5 * mp::m / mp::s, 2.0 * mp::m / mp::s,
      1.5 * mp::m / mp::s};

  const frc::MecanumDriveWheelSpeeds result = speeds * 2;

  EXPECT_EQ(2.0, mp::value(result.frontLeft));
  EXPECT_EQ(1.0, mp::value(result.frontRight));
  EXPECT_EQ(4.0, mp::value(result.rearLeft));
  EXPECT_EQ(3.0, mp::value(result.rearRight));
}

TEST(MecanumDriveWheelSpeedsTest, Division) {
  const frc::MecanumDriveWheelSpeeds speeds{
      1.0 * mp::m / mp::s, 0.5 * mp::m / mp::s, 2.0 * mp::m / mp::s,
      1.5 * mp::m / mp::s};

  const frc::MecanumDriveWheelSpeeds result = speeds / 2;

  EXPECT_EQ(0.5, mp::value(result.frontLeft));
  EXPECT_EQ(0.25, mp::value(result.frontRight));
  EXPECT_EQ(1.0, mp::value(result.rearLeft));
  EXPECT_EQ(0.75, mp::value(result.rearRight));
}
