// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/math/kinematics/MecanumDriveWheelAccelerations.hpp"

#include <gtest/gtest.h>

#include "wpi/units/acceleration.hpp"

using namespace wpi::math;

static constexpr double EPSILON = 1E-9;

TEST(MecanumDriveWheelAccelerationsTest, DefaultConstructor) {
  MecanumDriveWheelAccelerations wheelAccelerations;

  EXPECT_NEAR(wheelAccelerations.frontLeft.value(), 0.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.frontRight.value(), 0.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.rearLeft.value(), 0.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.rearRight.value(), 0.0, EPSILON);
}

TEST(MecanumDriveWheelAccelerationsTest, ParameterizedConstructor) {
  MecanumDriveWheelAccelerations wheelAccelerations{1.0_mps_sq, 2.0_mps_sq,
                                                    3.0_mps_sq, 4.0_mps_sq};

  EXPECT_NEAR(wheelAccelerations.frontLeft.value(), 1.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.frontRight.value(), 2.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.rearLeft.value(), 3.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.rearRight.value(), 4.0, EPSILON);
}

TEST(MecanumDriveWheelAccelerationsTest, Plus) {
  const MecanumDriveWheelAccelerations left{1.0_mps_sq, 0.5_mps_sq, 2.0_mps_sq,
                                            1.5_mps_sq};
  const MecanumDriveWheelAccelerations right{2.0_mps_sq, 1.5_mps_sq, 0.5_mps_sq,
                                             1.0_mps_sq};

  const auto wheelAccelerations = left + right;

  EXPECT_NEAR(wheelAccelerations.frontLeft.value(), 3.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.frontRight.value(), 2.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.rearLeft.value(), 2.5, EPSILON);
  EXPECT_NEAR(wheelAccelerations.rearRight.value(), 2.5, EPSILON);
}

TEST(MecanumDriveWheelAccelerationsTest, Minus) {
  const MecanumDriveWheelAccelerations left{5.0_mps_sq, 4.0_mps_sq, 6.0_mps_sq,
                                            2.5_mps_sq};
  const MecanumDriveWheelAccelerations right{1.0_mps_sq, 2.0_mps_sq, 3.0_mps_sq,
                                             0.5_mps_sq};

  const auto wheelAccelerations = left - right;

  EXPECT_NEAR(wheelAccelerations.frontLeft.value(), 4.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.frontRight.value(), 2.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.rearLeft.value(), 3.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.rearRight.value(), 2.0, EPSILON);
}

TEST(MecanumDriveWheelAccelerationsTest, UnaryMinus) {
  const auto wheelAccelerations = -MecanumDriveWheelAccelerations{
      1.0_mps_sq, -2.0_mps_sq, 3.0_mps_sq, -4.0_mps_sq};

  EXPECT_NEAR(wheelAccelerations.frontLeft.value(), -1.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.frontRight.value(), 2.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.rearLeft.value(), -3.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.rearRight.value(), 4.0, EPSILON);
}

TEST(MecanumDriveWheelAccelerationsTest, Multiplication) {
  const auto wheelAccelerations =
      MecanumDriveWheelAccelerations{2.0_mps_sq, 2.5_mps_sq, 3.0_mps_sq,
                                     3.5_mps_sq} *
      2.0;

  EXPECT_NEAR(wheelAccelerations.frontLeft.value(), 4.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.frontRight.value(), 5.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.rearLeft.value(), 6.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.rearRight.value(), 7.0, EPSILON);
}

TEST(MecanumDriveWheelAccelerationsTest, Division) {
  const auto wheelAccelerations =
      MecanumDriveWheelAccelerations{2.0_mps_sq, 2.5_mps_sq, 1.5_mps_sq,
                                     1.0_mps_sq} /
      2.0;

  EXPECT_NEAR(wheelAccelerations.frontLeft.value(), 1.0, EPSILON);
  EXPECT_NEAR(wheelAccelerations.frontRight.value(), 1.25, EPSILON);
  EXPECT_NEAR(wheelAccelerations.rearLeft.value(), 0.75, EPSILON);
  EXPECT_NEAR(wheelAccelerations.rearRight.value(), 0.5, EPSILON);
}
