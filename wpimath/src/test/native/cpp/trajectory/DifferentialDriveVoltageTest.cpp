// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h"
#include "frc/units-usc.h"
#include "frc/units.h"
#include "trajectory/TestTrajectory.h"

using namespace frc;

TEST(DifferentialDriveVoltageConstraintTest, Constraint) {
  // Pick an unreasonably large kA to ensure the constraint has to do some work
  SimpleMotorFeedforward<mp::m> feedforward{1.0 * mp::V,
                                            1.0 * mp::V / (mp::m / mp::s),
                                            3.0 * mp::V / (mp::m / mp::s2)};
  const DifferentialDriveKinematics kinematics{0.5 * mp::m};
  const auto maxVoltage = 10.0 * mp::V;

  auto config = TrajectoryConfig(12.0 * mp::ft / mp::s, 12.0 * mp::ft / mp::s2);
  config.AddConstraint(
      DifferentialDriveVoltageConstraint(feedforward, kinematics, maxVoltage));

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
    auto acceleration = point.acceleration;
    // Not really a strictly-correct test as we're using the chassis accel
    // instead of the wheel accel, but much easier than doing it "properly" and
    // a reasonable check anyway
    EXPECT_TRUE(feedforward.Calculate(left, left + acceleration * dt) <
                maxVoltage + 0.05 * mp::V);
    EXPECT_TRUE(feedforward.Calculate(left, left + acceleration * dt) >
                -maxVoltage - 0.05 * mp::V);
    EXPECT_TRUE(feedforward.Calculate(right,

                                      right + acceleration * dt) <
                maxVoltage + 0.05 * mp::V);
    EXPECT_TRUE(feedforward.Calculate(right, right + acceleration * dt) >
                -maxVoltage - 0.05 * mp::V);
  }
}

TEST(DifferentialDriveVoltageConstraintTest, HighCurvature) {
  SimpleMotorFeedforward<mp::m> feedforward{1.0 * mp::V,
                                            1.0 * mp::V / (mp::m / mp::s),
                                            3.0 * mp::V / (mp::m / mp::s2)};
  // Large trackwidth - need to test with radius of curvature less than half of
  // trackwidth
  const DifferentialDriveKinematics kinematics{3.0 * mp::m};
  const auto maxVoltage = 10.0 * mp::V;

  auto config = TrajectoryConfig(12.0 * mp::ft / mp::s, 12.0 * mp::ft / mp::s2);
  config.AddConstraint(
      DifferentialDriveVoltageConstraint(feedforward, kinematics, maxVoltage));

  EXPECT_NO_FATAL_FAILURE(TrajectoryGenerator::GenerateTrajectory(
      Pose2d{1.0 * mp::m, 0.0 * mp::m, 90.0 * mp::deg},
      std::vector<Translation2d>{},
      Pose2d{0.0 * mp::m, 1.0 * mp::m, 180.0 * mp::deg}, config));

  config.SetReversed(true);

  EXPECT_NO_FATAL_FAILURE(TrajectoryGenerator::GenerateTrajectory(
      Pose2d{0.0 * mp::m, 1.0 * mp::m, 180.0 * mp::deg},
      std::vector<Translation2d>{},
      Pose2d{1.0 * mp::m, 0.0 * mp::m, 90.0 * mp::deg}, config));
}
