// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <gtest/gtest.h>

#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/units.h"

using namespace frc;

static constexpr double kEpsilon = 1E-9;

TEST(DifferentialDriveKinematicsTest, InverseKinematicsFromZero) {
  const DifferentialDriveKinematics kinematics{2 * 0.381 * mp::m};
  const ChassisSpeeds chassisSpeeds;
  const auto wheelSpeeds = kinematics.ToWheelSpeeds(chassisSpeeds);

  EXPECT_NEAR(mp::value(wheelSpeeds.left), 0, kEpsilon);
  EXPECT_NEAR(mp::value(wheelSpeeds.right), 0, kEpsilon);
}

TEST(DifferentialDriveKinematicsTest, ForwardKinematicsFromZero) {
  const DifferentialDriveKinematics kinematics{2 * 0.381 * mp::m};
  const DifferentialDriveWheelSpeeds wheelSpeeds;
  const auto chassisSpeeds = kinematics.ToChassisSpeeds(wheelSpeeds);

  EXPECT_NEAR(mp::value(chassisSpeeds.vx), 0, kEpsilon);
  EXPECT_NEAR(mp::value(chassisSpeeds.vy), 0, kEpsilon);
  EXPECT_NEAR(mp::value(chassisSpeeds.omega), 0, kEpsilon);
}

TEST(DifferentialDriveKinematicsTest, InverseKinematicsForStraightLine) {
  const DifferentialDriveKinematics kinematics{2 * 0.381 * mp::m};
  const ChassisSpeeds chassisSpeeds{3.0 * mp::m / mp::s, 0.0 * mp::m / mp::s,
                                    0.0 * mp::rad / mp::s};
  const auto wheelSpeeds = kinematics.ToWheelSpeeds(chassisSpeeds);

  EXPECT_NEAR(mp::value(wheelSpeeds.left), 3, kEpsilon);
  EXPECT_NEAR(mp::value(wheelSpeeds.right), 3, kEpsilon);
}

TEST(DifferentialDriveKinematicsTest, ForwardKinematicsForStraightLine) {
  const DifferentialDriveKinematics kinematics{2 * 0.381 * mp::m};
  const DifferentialDriveWheelSpeeds wheelSpeeds{3.0 * mp::m / mp::s,
                                                 3.0 * mp::m / mp::s};
  const auto chassisSpeeds = kinematics.ToChassisSpeeds(wheelSpeeds);

  EXPECT_NEAR(mp::value(chassisSpeeds.vx), 3, kEpsilon);
  EXPECT_NEAR(mp::value(chassisSpeeds.vy), 0, kEpsilon);
  EXPECT_NEAR(mp::value(chassisSpeeds.omega), 0, kEpsilon);
}

TEST(DifferentialDriveKinematicsTest, InverseKinematicsForRotateInPlace) {
  const DifferentialDriveKinematics kinematics{2 * 0.381 * mp::m};
  const ChassisSpeeds chassisSpeeds{0.0 * mp::m / mp::s, 0.0 * mp::m / mp::s,
                                    std::numbers::pi * mp::rad / mp::s};
  const auto wheelSpeeds = kinematics.ToWheelSpeeds(chassisSpeeds);

  EXPECT_NEAR(mp::value(wheelSpeeds.left), -0.381 * std::numbers::pi, kEpsilon);
  EXPECT_NEAR(mp::value(wheelSpeeds.right), +0.381 * std::numbers::pi,
              kEpsilon);
}

TEST(DifferentialDriveKinematicsTest, ForwardKinematicsForRotateInPlace) {
  const DifferentialDriveKinematics kinematics{2 * 0.381 * mp::m};
  const DifferentialDriveWheelSpeeds wheelSpeeds{
      +0.381 * std::numbers::pi * mp::m / mp::s,
      -0.381 * std::numbers::pi * mp::m / mp::s};
  const auto chassisSpeeds = kinematics.ToChassisSpeeds(wheelSpeeds);

  EXPECT_NEAR(mp::value(chassisSpeeds.vx), 0, kEpsilon);
  EXPECT_NEAR(mp::value(chassisSpeeds.vy), 0, kEpsilon);
  EXPECT_NEAR(mp::value(chassisSpeeds.omega), -std::numbers::pi, kEpsilon);
}
