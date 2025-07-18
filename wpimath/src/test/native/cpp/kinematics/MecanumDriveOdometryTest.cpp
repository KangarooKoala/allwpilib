// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <limits>
#include <random>

#include <gtest/gtest.h>

#include "frc/kinematics/MecanumDriveOdometry.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/units.h"

using namespace frc;

class MecanumDriveOdometryTest : public ::testing::Test {
 protected:
  Translation2d m_fl{12.0 * mp::m, 12.0 * mp::m};
  Translation2d m_fr{12.0 * mp::m, -12.0 * mp::m};
  Translation2d m_bl{-12.0 * mp::m, 12.0 * mp::m};
  Translation2d m_br{-12.0 * mp::m, -12.0 * mp::m};

  MecanumDriveWheelPositions zero;

  MecanumDriveKinematics kinematics{m_fl, m_fr, m_bl, m_br};
  MecanumDriveOdometry odometry{kinematics, 0.0 * mp::rad, zero};
};

TEST_F(MecanumDriveOdometryTest, MultipleConsecutiveUpdates) {
  MecanumDriveWheelPositions wheelDeltas{3.536 * mp::m, 3.536 * mp::m,
                                         3.536 * mp::m, 3.536 * mp::m};

  odometry.ResetPosition(0.0 * mp::rad, wheelDeltas, Pose2d{});

  odometry.Update(0.0 * mp::deg, wheelDeltas);
  auto secondPose = odometry.Update(0.0 * mp::deg, wheelDeltas);

  EXPECT_NEAR(mp::value(secondPose.X()), 0.0, 0.01);
  EXPECT_NEAR(mp::value(secondPose.Y()), 0.0, 0.01);
  EXPECT_NEAR(mp::value(secondPose.Rotation().Radians()), 0.0, 0.01);
}

TEST_F(MecanumDriveOdometryTest, TwoIterations) {
  odometry.ResetPosition(0.0 * mp::rad, zero, Pose2d{});
  MecanumDriveWheelPositions wheelDeltas{0.3536 * mp::m, 0.3536 * mp::m,
                                         0.3536 * mp::m, 0.3536 * mp::m};

  odometry.Update(0.0 * mp::deg, MecanumDriveWheelPositions{});
  auto pose = odometry.Update(0.0 * mp::deg, wheelDeltas);

  EXPECT_NEAR(mp::value(pose.X()), 0.3536, 0.01);
  EXPECT_NEAR(mp::value(pose.Y()), 0.0, 0.01);
  EXPECT_NEAR(mp::value(pose.Rotation().Radians()), 0.0, 0.01);
}

TEST_F(MecanumDriveOdometryTest, 90DegreeTurn) {
  odometry.ResetPosition(0.0 * mp::rad, zero, Pose2d{});
  MecanumDriveWheelPositions wheelDeltas{-13.328 * mp::m, 39.986 * mp::m,
                                         -13.329 * mp::m, 39.986 * mp::m};
  odometry.Update(0.0 * mp::deg, MecanumDriveWheelPositions{});
  auto pose = odometry.Update(90.0 * mp::deg, wheelDeltas);

  EXPECT_NEAR(mp::value(pose.X()), 8.4855, 0.01);
  EXPECT_NEAR(mp::value(pose.Y()), 8.4855, 0.01);
  EXPECT_NEAR(mp::value(pose.Rotation().Degrees()), 90.0, 0.01);
}

TEST_F(MecanumDriveOdometryTest, GyroAngleReset) {
  odometry.ResetPosition(90.0 * mp::deg, zero, Pose2d{});

  MecanumDriveWheelPositions wheelDeltas{0.3536 * mp::m, 0.3536 * mp::m,
                                         0.3536 * mp::m, 0.3536 * mp::m};

  odometry.Update(90.0 * mp::deg, MecanumDriveWheelPositions{});
  auto pose = odometry.Update(90.0 * mp::deg, wheelDeltas);

  EXPECT_NEAR(mp::value(pose.X()), 0.3536, 0.01);
  EXPECT_NEAR(mp::value(pose.Y()), 0.0, 0.01);
  EXPECT_NEAR(mp::value(pose.Rotation().Radians()), 0.0, 0.01);
}

TEST_F(MecanumDriveOdometryTest, AccuracyFacingTrajectory) {
  frc::MecanumDriveKinematics kinematics{
      frc::Translation2d{1.0 * mp::m, 1.0 * mp::m},
      frc::Translation2d{1.0 * mp::m, -1.0 * mp::m},
      frc::Translation2d{-1.0 * mp::m, -1.0 * mp::m},
      frc::Translation2d{-1.0 * mp::m, 1.0 * mp::m}};

  frc::MecanumDriveWheelPositions wheelPositions;

  frc::MecanumDriveOdometry odometry{kinematics, frc::Rotation2d{},
                                     wheelPositions};

  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      std::vector{frc::Pose2d{0.0 * mp::m, 0.0 * mp::m, 45.0 * mp::deg},
                  frc::Pose2d{3.0 * mp::m, 0.0 * mp::m, -90.0 * mp::deg},
                  frc::Pose2d{0.0 * mp::m, 0.0 * mp::m, 135.0 * mp::deg},
                  frc::Pose2d{-3.0 * mp::m, 0.0 * mp::m, -90.0 * mp::deg},
                  frc::Pose2d{0.0 * mp::m, 0.0 * mp::m, 45.0 * mp::deg}},
      frc::TrajectoryConfig(5.0 * mp::m / mp::s, 2.0 * mp::m / mp::s2));

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 1.0);

  mp::quantity<mp::s> dt = 20.0 * mp::ms;
  mp::quantity<mp::s> t = 0.0 * mp::s;

  double maxError = -std::numeric_limits<double>::max();
  double errorSum = 0;

  while (t < trajectory.TotalTime()) {
    frc::Trajectory::State groundTruthState = trajectory.Sample(t);

    auto wheelSpeeds = kinematics.ToWheelSpeeds(
        {groundTruthState.velocity, 0.0 * mp::m / mp::s,
         groundTruthState.velocity * groundTruthState.curvature});

    wheelSpeeds.frontLeft += distribution(generator) * 0.1 * mp::m / mp::s;
    wheelSpeeds.frontRight += distribution(generator) * 0.1 * mp::m / mp::s;
    wheelSpeeds.rearLeft += distribution(generator) * 0.1 * mp::m / mp::s;
    wheelSpeeds.rearRight += distribution(generator) * 0.1 * mp::m / mp::s;

    wheelPositions.frontLeft += wheelSpeeds.frontLeft * dt;
    wheelPositions.frontRight += wheelSpeeds.frontRight * dt;
    wheelPositions.rearLeft += wheelSpeeds.rearLeft * dt;
    wheelPositions.rearRight += wheelSpeeds.rearRight * dt;

    auto xhat = odometry.Update(
        groundTruthState.pose.Rotation() +
            frc::Rotation2d{distribution(generator) * 0.05 * mp::rad},
        wheelPositions);
    double error = mp::value(
        groundTruthState.pose.Translation().Distance(xhat.Translation()));

    if (error > maxError) {
      maxError = error;
    }
    errorSum += error;

    t += dt;
  }

  EXPECT_LT(errorSum / double{trajectory.TotalTime() / dt}, 0.06);
  EXPECT_LT(maxError, 0.125);
}

TEST_F(MecanumDriveOdometryTest, AccuracyFacingXAxis) {
  frc::MecanumDriveKinematics kinematics{
      frc::Translation2d{1.0 * mp::m, 1.0 * mp::m},
      frc::Translation2d{1.0 * mp::m, -1.0 * mp::m},
      frc::Translation2d{-1.0 * mp::m, -1.0 * mp::m},
      frc::Translation2d{-1.0 * mp::m, 1.0 * mp::m}};

  frc::MecanumDriveWheelPositions wheelPositions;

  frc::MecanumDriveOdometry odometry{kinematics, frc::Rotation2d{},
                                     wheelPositions};

  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      std::vector{frc::Pose2d{0.0 * mp::m, 0.0 * mp::m, 45.0 * mp::deg},
                  frc::Pose2d{3.0 * mp::m, 0.0 * mp::m, -90.0 * mp::deg},
                  frc::Pose2d{0.0 * mp::m, 0.0 * mp::m, 135.0 * mp::deg},
                  frc::Pose2d{-3.0 * mp::m, 0.0 * mp::m, -90.0 * mp::deg},
                  frc::Pose2d{0.0 * mp::m, 0.0 * mp::m, 45.0 * mp::deg}},
      frc::TrajectoryConfig(5.0 * mp::m / mp::s, 2.0 * mp::m / mp::s2));

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 1.0);

  mp::quantity<mp::s> dt = 20.0 * mp::ms;
  mp::quantity<mp::s> t = 0.0 * mp::s;

  double maxError = -std::numeric_limits<double>::max();
  double errorSum = 0;

  while (t < trajectory.TotalTime()) {
    frc::Trajectory::State groundTruthState = trajectory.Sample(t);

    auto wheelSpeeds = kinematics.ToWheelSpeeds(
        {groundTruthState.velocity * groundTruthState.pose.Rotation().Cos(),
         groundTruthState.velocity * groundTruthState.pose.Rotation().Sin(),
         0.0 * mp::rad / mp::s});

    wheelSpeeds.frontLeft += distribution(generator) * 0.1 * mp::m / mp::s;
    wheelSpeeds.frontRight += distribution(generator) * 0.1 * mp::m / mp::s;
    wheelSpeeds.rearLeft += distribution(generator) * 0.1 * mp::m / mp::s;
    wheelSpeeds.rearRight += distribution(generator) * 0.1 * mp::m / mp::s;

    wheelPositions.frontLeft += wheelSpeeds.frontLeft * dt;
    wheelPositions.frontRight += wheelSpeeds.frontRight * dt;
    wheelPositions.rearLeft += wheelSpeeds.rearLeft * dt;
    wheelPositions.rearRight += wheelSpeeds.rearRight * dt;

    auto xhat = odometry.Update(
        frc::Rotation2d{distribution(generator) * 0.05 * mp::rad},
        wheelPositions);
    double error = mp::value(
        groundTruthState.pose.Translation().Distance(xhat.Translation()));

    if (error > maxError) {
      maxError = error;
    }
    errorSum += error;

    t += dt;
  }

  EXPECT_LT(errorSum / double{trajectory.TotalTime() / dt}, 0.06);
  EXPECT_LT(maxError, 0.125);
}
