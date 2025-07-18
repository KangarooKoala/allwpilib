// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <gtest/gtest.h>

#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/MecanumDriveKinematics.h"
#include "frc/units.h"

using namespace frc;

class MecanumDriveKinematicsTest : public ::testing::Test {
 protected:
  Translation2d m_fl{12.0 * mp::m, 12.0 * mp::m};
  Translation2d m_fr{12.0 * mp::m, -12.0 * mp::m};
  Translation2d m_bl{-12.0 * mp::m, 12.0 * mp::m};
  Translation2d m_br{-12.0 * mp::m, -12.0 * mp::m};

  MecanumDriveKinematics kinematics{m_fl, m_fr, m_bl, m_br};
};

TEST_F(MecanumDriveKinematicsTest, StraightLineInverseKinematics) {
  ChassisSpeeds speeds{5.0 * mp::m / mp::s, 0.0 * mp::m / mp::s,
                       0.0 * mp::rad / mp::s};
  auto moduleStates = kinematics.ToWheelSpeeds(speeds);

  EXPECT_NEAR(5.0, mp::value(moduleStates.frontLeft), 0.1);
  EXPECT_NEAR(5.0, mp::value(moduleStates.frontRight), 0.1);
  EXPECT_NEAR(5.0, mp::value(moduleStates.rearLeft), 0.1);
  EXPECT_NEAR(5.0, mp::value(moduleStates.rearRight), 0.1);
}

TEST_F(MecanumDriveKinematicsTest, StraightLineForwardKinematics) {
  MecanumDriveWheelSpeeds wheelSpeeds{5.0 * mp::m / mp::s, 5.0 * mp::m / mp::s,
                                      5.0 * mp::m / mp::s, 5.0 * mp::m / mp::s};
  auto chassisSpeeds = kinematics.ToChassisSpeeds(wheelSpeeds);

  EXPECT_NEAR(5.0, mp::value(chassisSpeeds.vx), 0.1);
  EXPECT_NEAR(0.0, mp::value(chassisSpeeds.vy), 0.1);
  EXPECT_NEAR(0.0, mp::value(chassisSpeeds.omega), 0.1);
}

TEST_F(MecanumDriveKinematicsTest, StraightLineForwardKinematicsWithDeltas) {
  MecanumDriveWheelPositions wheelDeltas{5.0 * mp::m, 5.0 * mp::m, 5.0 * mp::m,
                                         5.0 * mp::m};
  auto twist = kinematics.ToTwist2d(wheelDeltas);

  EXPECT_NEAR(5.0, mp::value(twist.dx), 0.1);
  EXPECT_NEAR(0.0, mp::value(twist.dy), 0.1);
  EXPECT_NEAR(0.0, mp::value(twist.dtheta), 0.1);
}

TEST_F(MecanumDriveKinematicsTest, StrafeInverseKinematics) {
  ChassisSpeeds speeds{0.0 * mp::m / mp::s, 4.0 * mp::m / mp::s,
                       0.0 * mp::rad / mp::s};
  auto moduleStates = kinematics.ToWheelSpeeds(speeds);

  EXPECT_NEAR(-4.0, mp::value(moduleStates.frontLeft), 0.1);
  EXPECT_NEAR(4.0, mp::value(moduleStates.frontRight), 0.1);
  EXPECT_NEAR(4.0, mp::value(moduleStates.rearLeft), 0.1);
  EXPECT_NEAR(-4.0, mp::value(moduleStates.rearRight), 0.1);
}

TEST_F(MecanumDriveKinematicsTest, StrafeForwardKinematics) {
  MecanumDriveWheelSpeeds wheelSpeeds{-5.0 * mp::m / mp::s, 5.0 * mp::m / mp::s,
                                      5.0 * mp::m / mp::s,
                                      -5.0 * mp::m / mp::s};
  auto chassisSpeeds = kinematics.ToChassisSpeeds(wheelSpeeds);

  EXPECT_NEAR(0.0, mp::value(chassisSpeeds.vx), 0.1);
  EXPECT_NEAR(5.0, mp::value(chassisSpeeds.vy), 0.1);
  EXPECT_NEAR(0.0, mp::value(chassisSpeeds.omega), 0.1);
}

TEST_F(MecanumDriveKinematicsTest, StrafeForwardKinematicsWithDeltas) {
  MecanumDriveWheelPositions wheelDeltas{-5.0 * mp::m, 5.0 * mp::m, 5.0 * mp::m,
                                         -5.0 * mp::m};
  auto twist = kinematics.ToTwist2d(wheelDeltas);

  EXPECT_NEAR(0.0, mp::value(twist.dx), 0.1);
  EXPECT_NEAR(5.0, mp::value(twist.dy), 0.1);
  EXPECT_NEAR(0.0, mp::value(twist.dtheta), 0.1);
}

TEST_F(MecanumDriveKinematicsTest, RotationInverseKinematics) {
  ChassisSpeeds speeds{0.0 * mp::m / mp::s, 0.0 * mp::m / mp::s,
                       2 * std::numbers::pi * mp::rad / mp::s};
  auto moduleStates = kinematics.ToWheelSpeeds(speeds);

  EXPECT_NEAR(-150.79644737, mp::value(moduleStates.frontLeft), 0.1);
  EXPECT_NEAR(150.79644737, mp::value(moduleStates.frontRight), 0.1);
  EXPECT_NEAR(-150.79644737, mp::value(moduleStates.rearLeft), 0.1);
  EXPECT_NEAR(150.79644737, mp::value(moduleStates.rearRight), 0.1);
}

TEST_F(MecanumDriveKinematicsTest, RotationForwardKinematics) {
  MecanumDriveWheelSpeeds wheelSpeeds{
      -150.79644737 * mp::m / mp::s, 150.79644737 * mp::m / mp::s,
      -150.79644737 * mp::m / mp::s, 150.79644737 * mp::m / mp::s};
  auto chassisSpeeds = kinematics.ToChassisSpeeds(wheelSpeeds);

  EXPECT_NEAR(0.0, mp::value(chassisSpeeds.vx), 0.1);
  EXPECT_NEAR(0.0, mp::value(chassisSpeeds.vy), 0.1);
  EXPECT_NEAR(2 * std::numbers::pi, mp::value(chassisSpeeds.omega), 0.1);
}

TEST_F(MecanumDriveKinematicsTest, RotationForwardKinematicsWithDeltas) {
  MecanumDriveWheelPositions wheelDeltas{
      -150.79644737 * mp::m, 150.79644737 * mp::m, -150.79644737 * mp::m,
      150.79644737 * mp::m};
  auto twist = kinematics.ToTwist2d(wheelDeltas);

  EXPECT_NEAR(0.0, mp::value(twist.dx), 0.1);
  EXPECT_NEAR(0.0, mp::value(twist.dy), 0.1);
  EXPECT_NEAR(2 * std::numbers::pi, mp::value(twist.dtheta), 0.1);
}

TEST_F(MecanumDriveKinematicsTest, MixedRotationTranslationInverseKinematics) {
  ChassisSpeeds speeds{2.0 * mp::m / mp::s, 3.0 * mp::m / mp::s,
                       1.0 * mp::rad / mp::s};
  auto moduleStates = kinematics.ToWheelSpeeds(speeds);

  EXPECT_NEAR(-25.0, mp::value(moduleStates.frontLeft), 0.1);
  EXPECT_NEAR(29.0, mp::value(moduleStates.frontRight), 0.1);
  EXPECT_NEAR(-19.0, mp::value(moduleStates.rearLeft), 0.1);
  EXPECT_NEAR(23.0, mp::value(moduleStates.rearRight), 0.1);
}

TEST_F(MecanumDriveKinematicsTest, MixedRotationTranslationForwardKinematics) {
  MecanumDriveWheelSpeeds wheelSpeeds{
      -17.677670 * mp::m / mp::s, 20.506097 * mp::m / mp::s,
      -13.435 * mp::m / mp::s, 16.26 * mp::m / mp::s};

  auto chassisSpeeds = kinematics.ToChassisSpeeds(wheelSpeeds);

  EXPECT_NEAR(1.41335, mp::value(chassisSpeeds.vx), 0.1);
  EXPECT_NEAR(2.1221, mp::value(chassisSpeeds.vy), 0.1);
  EXPECT_NEAR(0.707, mp::value(chassisSpeeds.omega), 0.1);
}

TEST_F(MecanumDriveKinematicsTest,
       MixedRotationTranslationForwardKinematicsWithDeltas) {
  MecanumDriveWheelPositions wheelDeltas{-17.677670 * mp::m, 20.506097 * mp::m,
                                         -13.435 * mp::m, 16.26 * mp::m};

  auto twist = kinematics.ToTwist2d(wheelDeltas);

  EXPECT_NEAR(1.41335, mp::value(twist.dx), 0.1);
  EXPECT_NEAR(2.1221, mp::value(twist.dy), 0.1);
  EXPECT_NEAR(0.707, mp::value(twist.dtheta), 0.1);
}

TEST_F(MecanumDriveKinematicsTest, OffCenterRotationInverseKinematics) {
  ChassisSpeeds speeds{0.0 * mp::m / mp::s, 0.0 * mp::m / mp::s,
                       1.0 * mp::rad / mp::s};
  auto moduleStates = kinematics.ToWheelSpeeds(speeds, m_fl);

  EXPECT_NEAR(0, mp::value(moduleStates.frontLeft), 0.1);
  EXPECT_NEAR(24.0, mp::value(moduleStates.frontRight), 0.1);
  EXPECT_NEAR(-24.0, mp::value(moduleStates.rearLeft), 0.1);
  EXPECT_NEAR(48.0, mp::value(moduleStates.rearRight), 0.1);
}

TEST_F(MecanumDriveKinematicsTest, OffCenterRotationForwardKinematics) {
  MecanumDriveWheelSpeeds wheelSpeeds{
      0.0 * mp::m / mp::s, 16.971 * mp::m / mp::s, -16.971 * mp::m / mp::s,
      33.941 * mp::m / mp::s};
  auto chassisSpeeds = kinematics.ToChassisSpeeds(wheelSpeeds);

  EXPECT_NEAR(8.48525, mp::value(chassisSpeeds.vx), 0.1);
  EXPECT_NEAR(-8.48525, mp::value(chassisSpeeds.vy), 0.1);
  EXPECT_NEAR(0.707, mp::value(chassisSpeeds.omega), 0.1);
}

TEST_F(MecanumDriveKinematicsTest,
       OffCenterRotationForwardKinematicsWithDeltas) {
  MecanumDriveWheelPositions wheelDeltas{0.0 * mp::m, 16.971 * mp::m,
                                         -16.971 * mp::m, 33.941 * mp::m};
  auto twist = kinematics.ToTwist2d(wheelDeltas);

  EXPECT_NEAR(8.48525, mp::value(twist.dx), 0.1);
  EXPECT_NEAR(-8.48525, mp::value(twist.dy), 0.1);
  EXPECT_NEAR(0.707, mp::value(twist.dtheta), 0.1);
}

TEST_F(MecanumDriveKinematicsTest,
       OffCenterTranslationRotationInverseKinematics) {
  ChassisSpeeds speeds{5.0 * mp::m / mp::s, 2.0 * mp::m / mp::s,
                       1.0 * mp::rad / mp::s};
  auto moduleStates = kinematics.ToWheelSpeeds(speeds, m_fl);

  EXPECT_NEAR(3.0, mp::value(moduleStates.frontLeft), 0.1);
  EXPECT_NEAR(31.0, mp::value(moduleStates.frontRight), 0.1);
  EXPECT_NEAR(-17.0, mp::value(moduleStates.rearLeft), 0.1);
  EXPECT_NEAR(51.0, mp::value(moduleStates.rearRight), 0.1);
}

TEST_F(MecanumDriveKinematicsTest,
       OffCenterTranslationRotationForwardKinematics) {
  MecanumDriveWheelSpeeds wheelSpeeds{
      2.12 * mp::m / mp::s, 21.92 * mp::m / mp::s, -12.02 * mp::m / mp::s,
      36.06 * mp::m / mp::s};
  auto chassisSpeeds = kinematics.ToChassisSpeeds(wheelSpeeds);

  EXPECT_NEAR(12.02, mp::value(chassisSpeeds.vx), 0.1);
  EXPECT_NEAR(-7.07, mp::value(chassisSpeeds.vy), 0.1);
  EXPECT_NEAR(0.707, mp::value(chassisSpeeds.omega), 0.1);
}

TEST_F(MecanumDriveKinematicsTest,
       OffCenterTranslationRotationForwardKinematicsWithDeltas) {
  MecanumDriveWheelPositions wheelDeltas{2.12 * mp::m, 21.92 * mp::m,
                                         -12.02 * mp::m, 36.06 * mp::m};
  auto twist = kinematics.ToTwist2d(wheelDeltas);

  EXPECT_NEAR(12.02, mp::value(twist.dx), 0.1);
  EXPECT_NEAR(-7.07, mp::value(twist.dy), 0.1);
  EXPECT_NEAR(0.707, mp::value(twist.dtheta), 0.1);
}

TEST_F(MecanumDriveKinematicsTest, Desaturate) {
  auto wheelSpeeds =
      MecanumDriveWheelSpeeds{5.0 * mp::m / mp::s, 6.0 * mp::m / mp::s,
                              4.0 * mp::m / mp::s, 7.0 * mp::m / mp::s}
          .Desaturate(5.5 * mp::m / mp::s);

  double kFactor = 5.5 / 7.0;

  EXPECT_NEAR(mp::value(wheelSpeeds.frontLeft), 5.0 * kFactor, 1E-9);
  EXPECT_NEAR(mp::value(wheelSpeeds.frontRight), 6.0 * kFactor, 1E-9);
  EXPECT_NEAR(mp::value(wheelSpeeds.rearLeft), 4.0 * kFactor, 1E-9);
  EXPECT_NEAR(mp::value(wheelSpeeds.rearRight), 7.0 * kFactor, 1E-9);
}

TEST_F(MecanumDriveKinematicsTest, DesaturateNegativeSpeeds) {
  auto wheelSpeeds =
      MecanumDriveWheelSpeeds{-5.0 * mp::m / mp::s, 6.0 * mp::m / mp::s,
                              4.0 * mp::m / mp::s, -7.0 * mp::m / mp::s}
          .Desaturate(5.5 * mp::m / mp::s);

  constexpr double kFactor = 5.5 / 7.0;

  EXPECT_NEAR(mp::value(wheelSpeeds.frontLeft), -5.0 * kFactor, 1E-9);
  EXPECT_NEAR(mp::value(wheelSpeeds.frontRight), 6.0 * kFactor, 1E-9);
  EXPECT_NEAR(mp::value(wheelSpeeds.rearLeft), 4.0 * kFactor, 1E-9);
  EXPECT_NEAR(mp::value(wheelSpeeds.rearRight), -7.0 * kFactor, 1E-9);
}
