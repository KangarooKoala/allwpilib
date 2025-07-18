// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <gtest/gtest.h>

#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/kinematics/DifferentialDriveOdometry.h"
#include "frc/units.h"

static constexpr double kEpsilon = 1E-9;

using namespace frc;

TEST(DifferentialDriveOdometryTest, EncoderDistances) {
  DifferentialDriveOdometry odometry{45.0 * mp::deg, 0.0 * mp::m, 0.0 * mp::m};

  const auto& pose = odometry.Update(135.0 * mp::deg, 0.0 * mp::m,
                                     5.0 * std::numbers::pi * mp::m);

  EXPECT_NEAR(mp::value(pose.X()), 5.0, kEpsilon);
  EXPECT_NEAR(mp::value(pose.Y()), 5.0, kEpsilon);
  EXPECT_NEAR(mp::value(pose.Rotation().Degrees()), 90.0, kEpsilon);
}
