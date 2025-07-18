// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <gtest/gtest.h>

#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/units.h"

using namespace frc;

static constexpr double kEpsilon = 0.1;

class SwerveDriveKinematicsTest : public ::testing::Test {
 protected:
  Translation2d m_fl{12.0 * mp::m, 12.0 * mp::m};
  Translation2d m_fr{12.0 * mp::m, -12.0 * mp::m};
  Translation2d m_bl{-12.0 * mp::m, 12.0 * mp::m};
  Translation2d m_br{-12.0 * mp::m, -12.0 * mp::m};

  SwerveDriveKinematics<4> m_kinematics{m_fl, m_fr, m_bl, m_br};
};

TEST_F(SwerveDriveKinematicsTest, StraightLineInverseKinematics) {
  ChassisSpeeds speeds{5.0 * mp::m / mp::s, 0.0 * mp::m / mp::s,
                       0.0 * mp::rad / mp::s};

  auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds);

  EXPECT_NEAR(mp::value(fl.speed), 5.0, kEpsilon);
  EXPECT_NEAR(mp::value(fr.speed), 5.0, kEpsilon);
  EXPECT_NEAR(mp::value(bl.speed), 5.0, kEpsilon);
  EXPECT_NEAR(mp::value(br.speed), 5.0, kEpsilon);

  EXPECT_NEAR(mp::value(fl.angle.Radians()), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(fr.angle.Radians()), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(bl.angle.Radians()), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(br.angle.Radians()), 0.0, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, StraightLineForwardKinematics) {
  SwerveModuleState state{5.0 * mp::m / mp::s, 0.0 * mp::deg};

  auto chassisSpeeds = m_kinematics.ToChassisSpeeds(state, state, state, state);

  EXPECT_NEAR(mp::value(chassisSpeeds.vx), 5.0, kEpsilon);
  EXPECT_NEAR(mp::value(chassisSpeeds.vy), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(chassisSpeeds.omega), 0.0, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, StraightLineForwardKinematicsWithDeltas) {
  SwerveModulePosition delta{5.0 * mp::m, 0.0 * mp::deg};

  auto twist = m_kinematics.ToTwist2d(delta, delta, delta, delta);

  EXPECT_NEAR(mp::value(twist.dx), 5.0, kEpsilon);
  EXPECT_NEAR(mp::value(twist.dy), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(twist.dtheta), 0.0, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, StraightStrafeInverseKinematics) {
  ChassisSpeeds speeds{0.0 * mp::m / mp::s, 5.0 * mp::m / mp::s,
                       0.0 * mp::rad / mp::s};
  auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds);

  EXPECT_NEAR(mp::value(fl.speed), 5.0, kEpsilon);
  EXPECT_NEAR(mp::value(fr.speed), 5.0, kEpsilon);
  EXPECT_NEAR(mp::value(bl.speed), 5.0, kEpsilon);
  EXPECT_NEAR(mp::value(br.speed), 5.0, kEpsilon);

  EXPECT_NEAR(mp::value(fl.angle.Degrees()), 90.0, kEpsilon);
  EXPECT_NEAR(mp::value(fr.angle.Degrees()), 90.0, kEpsilon);
  EXPECT_NEAR(mp::value(bl.angle.Degrees()), 90.0, kEpsilon);
  EXPECT_NEAR(mp::value(br.angle.Degrees()), 90.0, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, StraightStrafeForwardKinematics) {
  SwerveModuleState state{5.0 * mp::m / mp::s, 90.0 * mp::deg};
  auto chassisSpeeds = m_kinematics.ToChassisSpeeds(state, state, state, state);

  EXPECT_NEAR(mp::value(chassisSpeeds.vx), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(chassisSpeeds.vy), 5.0, kEpsilon);
  EXPECT_NEAR(mp::value(chassisSpeeds.omega), 0.0, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, StraightStrafeForwardKinematicsWithDeltas) {
  SwerveModulePosition delta{5.0 * mp::m, 90.0 * mp::deg};

  auto twist = m_kinematics.ToTwist2d(delta, delta, delta, delta);

  EXPECT_NEAR(mp::value(twist.dx), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(twist.dy), 5.0, kEpsilon);
  EXPECT_NEAR(mp::value(twist.dtheta), 0.0, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, TurnInPlaceInverseKinematics) {
  ChassisSpeeds speeds{0.0 * mp::m / mp::s, 0.0 * mp::m / mp::s,
                       2 * std::numbers::pi * mp::rad / mp::s};
  auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds);

  EXPECT_NEAR(mp::value(fl.speed), 106.63, kEpsilon);
  EXPECT_NEAR(mp::value(fr.speed), 106.63, kEpsilon);
  EXPECT_NEAR(mp::value(bl.speed), 106.63, kEpsilon);
  EXPECT_NEAR(mp::value(br.speed), 106.63, kEpsilon);

  EXPECT_NEAR(mp::value(fl.angle.Degrees()), 135.0, kEpsilon);
  EXPECT_NEAR(mp::value(fr.angle.Degrees()), 45.0, kEpsilon);
  EXPECT_NEAR(mp::value(bl.angle.Degrees()), -135.0, kEpsilon);
  EXPECT_NEAR(mp::value(br.angle.Degrees()), -45.0, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, ConserveWheelAngle) {
  ChassisSpeeds speeds{0.0 * mp::m / mp::s, 0.0 * mp::m / mp::s,
                       2 * std::numbers::pi * mp::rad / mp::s};
  m_kinematics.ToSwerveModuleStates(speeds);
  auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(ChassisSpeeds{});

  EXPECT_NEAR(mp::value(fl.speed), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(fr.speed), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(bl.speed), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(br.speed), 0.0, kEpsilon);

  EXPECT_NEAR(mp::value(fl.angle.Degrees()), 135.0, kEpsilon);
  EXPECT_NEAR(mp::value(fr.angle.Degrees()), 45.0, kEpsilon);
  EXPECT_NEAR(mp::value(bl.angle.Degrees()), -135.0, kEpsilon);
  EXPECT_NEAR(mp::value(br.angle.Degrees()), -45.0, kEpsilon);
}
TEST_F(SwerveDriveKinematicsTest, ResetWheelAngle) {
  Rotation2d fl = {0.0 * mp::deg};
  Rotation2d fr = {90.0 * mp::deg};
  Rotation2d bl = {180.0 * mp::deg};
  Rotation2d br = {270.0 * mp::deg};
  m_kinematics.ResetHeadings(fl, fr, bl, br);
  auto [flMod, frMod, blMod, brMod] =
      m_kinematics.ToSwerveModuleStates(ChassisSpeeds{});

  EXPECT_NEAR(mp::value(flMod.speed), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(frMod.speed), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(blMod.speed), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(brMod.speed), 0.0, kEpsilon);

  EXPECT_NEAR(mp::value(flMod.angle.Degrees()), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(frMod.angle.Degrees()), 90.0, kEpsilon);
  EXPECT_NEAR(mp::value(blMod.angle.Degrees()), 180.0, kEpsilon);
  EXPECT_NEAR(mp::value(brMod.angle.Degrees()), -90.0, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, TurnInPlaceForwardKinematics) {
  SwerveModuleState fl{106.629 * mp::m / mp::s, 135.0 * mp::deg};
  SwerveModuleState fr{106.629 * mp::m / mp::s, 45.0 * mp::deg};
  SwerveModuleState bl{106.629 * mp::m / mp::s, -135.0 * mp::deg};
  SwerveModuleState br{106.629 * mp::m / mp::s, -45.0 * mp::deg};

  auto chassisSpeeds = m_kinematics.ToChassisSpeeds(fl, fr, bl, br);

  EXPECT_NEAR(mp::value(chassisSpeeds.vx), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(chassisSpeeds.vy), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(chassisSpeeds.omega), 2 * std::numbers::pi, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, TurnInPlaceForwardKinematicsWithDeltas) {
  SwerveModulePosition fl{106.629 * mp::m, 135.0 * mp::deg};
  SwerveModulePosition fr{106.629 * mp::m, 45.0 * mp::deg};
  SwerveModulePosition bl{106.629 * mp::m, -135.0 * mp::deg};
  SwerveModulePosition br{106.629 * mp::m, -45.0 * mp::deg};

  auto twist = m_kinematics.ToTwist2d(fl, fr, bl, br);

  EXPECT_NEAR(mp::value(twist.dx), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(twist.dy), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(twist.dtheta), 2 * std::numbers::pi, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, OffCenterCORRotationInverseKinematics) {
  ChassisSpeeds speeds{0.0 * mp::m / mp::s, 0.0 * mp::m / mp::s,
                       2 * std::numbers::pi * mp::rad / mp::s};
  auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds, m_fl);

  EXPECT_NEAR(mp::value(fl.speed), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(fr.speed), 150.796, kEpsilon);
  EXPECT_NEAR(mp::value(bl.speed), 150.796, kEpsilon);
  EXPECT_NEAR(mp::value(br.speed), 213.258, kEpsilon);

  EXPECT_NEAR(mp::value(fl.angle.Degrees()), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(fr.angle.Degrees()), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(bl.angle.Degrees()), -90.0, kEpsilon);
  EXPECT_NEAR(mp::value(br.angle.Degrees()), -45.0, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, OffCenterCORRotationForwardKinematics) {
  SwerveModuleState fl{0.0 * mp::m / mp::s, 0.0 * mp::deg};
  SwerveModuleState fr{150.796 * mp::m / mp::s, 0.0 * mp::deg};
  SwerveModuleState bl{150.796 * mp::m / mp::s, -90.0 * mp::deg};
  SwerveModuleState br{213.258 * mp::m / mp::s, -45.0 * mp::deg};

  auto chassisSpeeds = m_kinematics.ToChassisSpeeds(fl, fr, bl, br);

  EXPECT_NEAR(mp::value(chassisSpeeds.vx), 75.398, kEpsilon);
  EXPECT_NEAR(mp::value(chassisSpeeds.vy), -75.398, kEpsilon);
  EXPECT_NEAR(mp::value(chassisSpeeds.omega), 2 * std::numbers::pi, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest,
       OffCenterCORRotationForwardKinematicsWithDeltas) {
  SwerveModulePosition fl{0.0 * mp::m, 0.0 * mp::deg};
  SwerveModulePosition fr{150.796 * mp::m, 0.0 * mp::deg};
  SwerveModulePosition bl{150.796 * mp::m, -90.0 * mp::deg};
  SwerveModulePosition br{213.258 * mp::m, -45.0 * mp::deg};

  auto twist = m_kinematics.ToTwist2d(fl, fr, bl, br);

  EXPECT_NEAR(mp::value(twist.dx), 75.398, kEpsilon);
  EXPECT_NEAR(mp::value(twist.dy), -75.398, kEpsilon);
  EXPECT_NEAR(mp::value(twist.dtheta), 2 * std::numbers::pi, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest,
       OffCenterCORRotationAndTranslationInverseKinematics) {
  ChassisSpeeds speeds{0.0 * mp::m / mp::s, 3.0 * mp::m / mp::s,
                       1.5 * mp::rad / mp::s};
  auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(
      speeds, Translation2d{24.0 * mp::m, 0.0 * mp::m});

  EXPECT_NEAR(mp::value(fl.speed), 23.43, kEpsilon);
  EXPECT_NEAR(mp::value(fr.speed), 23.43, kEpsilon);
  EXPECT_NEAR(mp::value(bl.speed), 54.08, kEpsilon);
  EXPECT_NEAR(mp::value(br.speed), 54.08, kEpsilon);

  EXPECT_NEAR(mp::value(fl.angle.Degrees()), -140.19, kEpsilon);
  EXPECT_NEAR(mp::value(fr.angle.Degrees()), -39.81, kEpsilon);
  EXPECT_NEAR(mp::value(bl.angle.Degrees()), -109.44, kEpsilon);
  EXPECT_NEAR(mp::value(br.angle.Degrees()), -70.56, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest,
       OffCenterCORRotationAndTranslationForwardKinematics) {
  SwerveModuleState fl{23.43 * mp::m / mp::s, -140.19 * mp::deg};
  SwerveModuleState fr{23.43 * mp::m / mp::s, -39.81 * mp::deg};
  SwerveModuleState bl{54.08 * mp::m / mp::s, -109.44 * mp::deg};
  SwerveModuleState br{54.08 * mp::m / mp::s, -70.56 * mp::deg};

  auto chassisSpeeds = m_kinematics.ToChassisSpeeds(fl, fr, bl, br);

  EXPECT_NEAR(mp::value(chassisSpeeds.vx), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(chassisSpeeds.vy), -33.0, kEpsilon);
  EXPECT_NEAR(mp::value(chassisSpeeds.omega), 1.5, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest,
       OffCenterCORRotationAndTranslationForwardKinematicsWithDeltas) {
  SwerveModulePosition fl{23.43 * mp::m, -140.19 * mp::deg};
  SwerveModulePosition fr{23.43 * mp::m, -39.81 * mp::deg};
  SwerveModulePosition bl{54.08 * mp::m, -109.44 * mp::deg};
  SwerveModulePosition br{54.08 * mp::m, -70.56 * mp::deg};

  auto twist = m_kinematics.ToTwist2d(fl, fr, bl, br);

  EXPECT_NEAR(mp::value(twist.dx), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(twist.dy), -33.0, kEpsilon);
  EXPECT_NEAR(mp::value(twist.dtheta), 1.5, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, Desaturate) {
  SwerveModuleState state1{5.0 * mp::m / mp::s, 0.0 * mp::deg};
  SwerveModuleState state2{6.0 * mp::m / mp::s, 0.0 * mp::deg};
  SwerveModuleState state3{4.0 * mp::m / mp::s, 0.0 * mp::deg};
  SwerveModuleState state4{7.0 * mp::m / mp::s, 0.0 * mp::deg};

  wpi::array<SwerveModuleState, 4> arr{state1, state2, state3, state4};
  SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&arr, 5.5 * mp::m / mp::s);

  double kFactor = 5.5 / 7.0;

  EXPECT_NEAR(mp::value(arr[0].speed), 5.0 * kFactor, kEpsilon);
  EXPECT_NEAR(mp::value(arr[1].speed), 6.0 * kFactor, kEpsilon);
  EXPECT_NEAR(mp::value(arr[2].speed), 4.0 * kFactor, kEpsilon);
  EXPECT_NEAR(mp::value(arr[3].speed), 7.0 * kFactor, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, DesaturateSmooth) {
  SwerveModuleState state1{5.0 * mp::m / mp::s, 0.0 * mp::deg};
  SwerveModuleState state2{6.0 * mp::m / mp::s, 0.0 * mp::deg};
  SwerveModuleState state3{4.0 * mp::m / mp::s, 0.0 * mp::deg};
  SwerveModuleState state4{7.0 * mp::m / mp::s, 0.0 * mp::deg};

  wpi::array<SwerveModuleState, 4> arr{state1, state2, state3, state4};
  SwerveDriveKinematics<4>::DesaturateWheelSpeeds(
      &arr, m_kinematics.ToChassisSpeeds(arr), 5.5 * mp::m / mp::s,
      5.5 * mp::m / mp::s, 3.5 * mp::rad / mp::s);

  double kFactor = 5.5 / 7.0;

  EXPECT_NEAR(mp::value(arr[0].speed), 5.0 * kFactor, kEpsilon);
  EXPECT_NEAR(mp::value(arr[1].speed), 6.0 * kFactor, kEpsilon);
  EXPECT_NEAR(mp::value(arr[2].speed), 4.0 * kFactor, kEpsilon);
  EXPECT_NEAR(mp::value(arr[3].speed), 7.0 * kFactor, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, DesaturateNegativeSpeed) {
  SwerveModuleState state1{1.0 * mp::m / mp::s, 0.0 * mp::deg};
  SwerveModuleState state2{1.0 * mp::m / mp::s, 0.0 * mp::deg};
  SwerveModuleState state3{-2.0 * mp::m / mp::s, 0.0 * mp::deg};
  SwerveModuleState state4{-2.0 * mp::m / mp::s, 0.0 * mp::deg};

  wpi::array<SwerveModuleState, 4> arr{state1, state2, state3, state4};
  SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&arr, 1.0 * mp::m / mp::s);

  EXPECT_NEAR(mp::value(arr[0].speed), 0.5, kEpsilon);
  EXPECT_NEAR(mp::value(arr[1].speed), 0.5, kEpsilon);
  EXPECT_NEAR(mp::value(arr[2].speed), -1.0, kEpsilon);
  EXPECT_NEAR(mp::value(arr[3].speed), -1.0, kEpsilon);
}
