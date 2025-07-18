// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <gtest/gtest.h>

#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/kinematics/DifferentialDriveOdometry3d.h"
#include "frc/units.h"

static constexpr double kEpsilon = 1E-9;

using namespace frc;

TEST(DifferentialDriveOdometry3dTest, Initialize) {
  DifferentialDriveOdometry3d odometry{
      frc::Rotation3d{0.0 * mp::deg, 0.0 * mp::deg, 90.0 * mp::deg},
      0.0 * mp::m, 0.0 * mp::m,
      frc::Pose3d{
          1.0 * mp::m, 2.0 * mp::m, 0.0 * mp::m,
          frc::Rotation3d{0.0 * mp::deg, 0.0 * mp::deg, 45.0 * mp::deg}}};

  const frc::Pose3d& pose = odometry.GetPose();

  EXPECT_NEAR(mp::value(pose.X()), 1, kEpsilon);
  EXPECT_NEAR(mp::value(pose.Y()), 2, kEpsilon);
  EXPECT_NEAR(mp::value(pose.Z()), 0, kEpsilon);
  EXPECT_NEAR(mp::value(pose.Rotation().ToRotation2d().Degrees()), 45,
              kEpsilon);
}

TEST(DifferentialDriveOdometry3dTest, EncoderDistances) {
  DifferentialDriveOdometry3d odometry{
      frc::Rotation3d{0.0 * mp::deg, 0.0 * mp::deg, 45.0 * mp::deg},
      0.0 * mp::m, 0.0 * mp::m};

  const auto& pose = odometry.Update(
      frc::Rotation3d{0.0 * mp::deg, 0.0 * mp::deg, 135.0 * mp::deg},
      0.0 * mp::m, 5.0 * std::numbers::pi * mp::m);

  EXPECT_NEAR(mp::value(pose.X()), 5.0, kEpsilon);
  EXPECT_NEAR(mp::value(pose.Y()), 5.0, kEpsilon);
  EXPECT_NEAR(mp::value(pose.Z()), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(pose.Rotation().ToRotation2d().Degrees()), 90.0,
              kEpsilon);
}
