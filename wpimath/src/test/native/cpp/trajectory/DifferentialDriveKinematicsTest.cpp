// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h"
#include "frc/units-usc.h"
#include "frc/units.h"
#include "trajectory/TestTrajectory.h"

using namespace frc;

TEST(DifferentialDriveKinematicsConstraintTest, Constraint) {
  const auto maxVelocity = 12.0 * mp::ft / mp::s;
  const DifferentialDriveKinematics kinematics{27.0 * mp::in};

  auto config = TrajectoryConfig(12.0 * mp::ft / mp::s, 12.0 * mp::ft / mp::s2);
  config.AddConstraint(
      DifferentialDriveKinematicsConstraint(kinematics, maxVelocity));

  auto trajectory = TestTrajectory::GetTrajectory(config);

  mp::quantity<mp::s> time = 0.0 * mp::s;
  mp::quantity<mp::s> dt = 20.0 * mp::ms;
  mp::quantity<mp::s> duration = trajectory.TotalTime();

  while (time < duration) {
    const Trajectory::State point = trajectory.Sample(time);
    time += dt;

    const ChassisSpeeds chassisSpeeds{point.velocity, 0.0 * mp::m / mp::s,
                                      point.velocity * point.curvature};

    auto [left, right] = kinematics.ToWheelSpeeds(chassisSpeeds);

    EXPECT_TRUE(left < maxVelocity + 0.05 * mp::m / mp::s);
    EXPECT_TRUE(right < maxVelocity + 0.05 * mp::m / mp::s);
  }
}
