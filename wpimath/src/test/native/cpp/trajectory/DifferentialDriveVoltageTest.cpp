// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <wpi/units/acceleration.h>
#include <wpi/units/length.h>
#include <wpi/units/time.h>
#include <wpi/units/velocity.h>
#include <wpi/units/voltage.h>

#include "wpi/math/geometry/Pose2d.hpp"
#include "wpi/math/kinematics/DifferentialDriveKinematics.hpp"
#include "wpi/math/trajectory/TestTrajectory.hpp"
#include "wpi/math/trajectory/TrajectoryGenerator.hpp"
#include "wpi/math/trajectory/constraint/DifferentialDriveVoltageConstraint.hpp"

using namespace wpi::math;

TEST(DifferentialDriveVoltageConstraintTest, Constraint) {
  // Pick an unreasonably large kA to ensure the constraint has to do some work
  SimpleMotorFeedforward<wpi::units::meters_> feedforward{1_V, 1_V / 1_mps,
                                                          3_V / 1_mps2};
  const DifferentialDriveKinematics kinematics{0.5_m};
  const auto maxVoltage = 10_V;

  auto config = TrajectoryConfig(12_fps, 12_fps2);
  config.AddConstraint(
      DifferentialDriveVoltageConstraint(feedforward, kinematics, maxVoltage));

  auto trajectory = TestTrajectory::GetTrajectory(config);

  wpi::units::seconds<> time = 0_s;
  wpi::units::seconds<> dt = 20_ms;
  wpi::units::seconds<> duration = trajectory.TotalTime();

  while (time < duration) {
    const Trajectory::State point = trajectory.Sample(time);
    time += dt;

    const ChassisSpeeds chassisSpeeds{point.velocity, 0_mps,
                                      point.velocity * point.curvature};

    auto [left, right] = kinematics.ToWheelSpeeds(chassisSpeeds);
    auto acceleration = point.acceleration;
    // Not really a strictly-correct test as we're using the chassis accel
    // instead of the wheel accel, but much easier than doing it "properly" and
    // a reasonable check anyway
    EXPECT_TRUE(feedforward.Calculate(left, left + acceleration * dt) <
                maxVoltage + 0.05_V);
    EXPECT_TRUE(feedforward.Calculate(left, left + acceleration * dt) >
                -maxVoltage - 0.05_V);
    EXPECT_TRUE(feedforward.Calculate(right,

                                      right + acceleration * dt) <
                maxVoltage + 0.05_V);
    EXPECT_TRUE(feedforward.Calculate(right, right + acceleration * dt) >
                -maxVoltage - 0.05_V);
  }
}

TEST(DifferentialDriveVoltageConstraintTest, HighCurvature) {
  SimpleMotorFeedforward<wpi::units::meters_> feedforward{1_V, 1_V / 1_mps,
                                                          3_V / 1_mps2};
  // Large trackwidth - need to test with radius of curvature less than half of
  // trackwidth
  const DifferentialDriveKinematics kinematics{3_m};
  const auto maxVoltage = 10_V;

  auto config = TrajectoryConfig(12_fps, 12_fps2);
  config.AddConstraint(
      DifferentialDriveVoltageConstraint(feedforward, kinematics, maxVoltage));

  EXPECT_NO_FATAL_FAILURE(TrajectoryGenerator::GenerateTrajectory(
      Pose2d{1_m, 0_m, 90_deg}, std::vector<Translation2d>{},
      Pose2d{0_m, 1_m, 180_deg}, config));

  config.SetReversed(true);

  EXPECT_NO_FATAL_FAILURE(TrajectoryGenerator::GenerateTrajectory(
      Pose2d{0_m, 1_m, 180_deg}, std::vector<Translation2d>{},
      Pose2d{1_m, 0_m, 90_deg}, config));
}
