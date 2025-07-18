// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <limits>
#include <random>

#include <gtest/gtest.h>

#include "frc/kinematics/MecanumDriveOdometry3d.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/units.h"

using namespace frc;

class MecanumDriveOdometry3dTest : public ::testing::Test {
 protected:
  Translation2d m_fl{12.0 * mp::m, 12.0 * mp::m};
  Translation2d m_fr{12.0 * mp::m, -12.0 * mp::m};
  Translation2d m_bl{-12.0 * mp::m, 12.0 * mp::m};
  Translation2d m_br{-12.0 * mp::m, -12.0 * mp::m};

  MecanumDriveWheelPositions zero;

  MecanumDriveKinematics kinematics{m_fl, m_fr, m_bl, m_br};
  MecanumDriveOdometry3d odometry{kinematics, frc::Rotation3d{}, zero};
};

TEST_F(MecanumDriveOdometry3dTest, Initialize) {
  MecanumDriveOdometry3d odometry{
      kinematics, frc::Rotation3d{}, zero,
      frc::Pose3d{
          1.0 * mp::m, 2.0 * mp::m, 0.0 * mp::m,
          frc::Rotation3d{0.0 * mp::deg, 0.0 * mp::deg, 45.0 * mp::deg}}};

  const frc::Pose3d& pose = odometry.GetPose();

  EXPECT_NEAR(mp::value(pose.X()), 1, 1e-9);
  EXPECT_NEAR(mp::value(pose.Y()), 2, 1e-9);
  EXPECT_NEAR(mp::value(pose.Z()), 0, 1e-9);
  EXPECT_NEAR(mp::value(pose.Rotation().ToRotation2d().Degrees()), 45, 1e-9);
}

TEST_F(MecanumDriveOdometry3dTest, MultipleConsecutiveUpdates) {
  MecanumDriveWheelPositions wheelDeltas{3.536 * mp::m, 3.536 * mp::m,
                                         3.536 * mp::m, 3.536 * mp::m};

  odometry.ResetPosition(frc::Rotation3d{}, wheelDeltas, Pose3d{});

  odometry.Update(frc::Rotation3d{}, wheelDeltas);
  auto secondPose = odometry.Update(frc::Rotation3d{}, wheelDeltas);

  EXPECT_NEAR(mp::value(secondPose.X()), 0.0, 0.01);
  EXPECT_NEAR(mp::value(secondPose.Y()), 0.0, 0.01);
  EXPECT_NEAR(mp::value(secondPose.Z()), 0.0, 0.01);
  EXPECT_NEAR(mp::value(secondPose.Rotation().ToRotation2d().Radians()), 0.0,
              0.01);
}

TEST_F(MecanumDriveOdometry3dTest, TwoIterations) {
  odometry.ResetPosition(frc::Rotation3d{}, zero, Pose3d{});
  MecanumDriveWheelPositions wheelDeltas{0.3536 * mp::m, 0.3536 * mp::m,
                                         0.3536 * mp::m, 0.3536 * mp::m};

  odometry.Update(frc::Rotation3d{}, MecanumDriveWheelPositions{});
  auto pose = odometry.Update(frc::Rotation3d{}, wheelDeltas);

  EXPECT_NEAR(mp::value(pose.X()), 0.3536, 0.01);
  EXPECT_NEAR(mp::value(pose.Y()), 0.0, 0.01);
  EXPECT_NEAR(mp::value(pose.Z()), 0.0, 0.01);
  EXPECT_NEAR(mp::value(pose.Rotation().ToRotation2d().Radians()), 0.0, 0.01);
}

TEST_F(MecanumDriveOdometry3dTest, 90DegreeTurn) {
  odometry.ResetPosition(frc::Rotation3d{}, zero, Pose3d{});
  MecanumDriveWheelPositions wheelDeltas{-13.328 * mp::m, 39.986 * mp::m,
                                         -13.329 * mp::m, 39.986 * mp::m};
  odometry.Update(frc::Rotation3d{}, MecanumDriveWheelPositions{});
  auto pose = odometry.Update(
      frc::Rotation3d{0.0 * mp::deg, 0.0 * mp::deg, 90.0 * mp::deg},
      wheelDeltas);

  EXPECT_NEAR(mp::value(pose.X()), 8.4855, 0.01);
  EXPECT_NEAR(mp::value(pose.Y()), 8.4855, 0.01);
  EXPECT_NEAR(mp::value(pose.Z()), 0, 0.01);
  EXPECT_NEAR(mp::value(pose.Rotation().ToRotation2d().Degrees()), 90.0, 0.01);
}

TEST_F(MecanumDriveOdometry3dTest, GyroAngleReset) {
  odometry.ResetPosition(
      frc::Rotation3d{0.0 * mp::deg, 0.0 * mp::deg, 90.0 * mp::deg}, zero,
      Pose3d{});

  MecanumDriveWheelPositions wheelDeltas{0.3536 * mp::m, 0.3536 * mp::m,
                                         0.3536 * mp::m, 0.3536 * mp::m};

  auto pose = odometry.Update(
      frc::Rotation3d{0.0 * mp::deg, 0.0 * mp::deg, 90.0 * mp::deg},
      wheelDeltas);

  EXPECT_NEAR(mp::value(pose.X()), 0.3536, 0.01);
  EXPECT_NEAR(mp::value(pose.Y()), 0.0, 0.01);
  EXPECT_NEAR(mp::value(pose.Z()), 0.0, 0.01);
  EXPECT_NEAR(mp::value(pose.Rotation().ToRotation2d().Radians()), 0.0, 0.01);
}

TEST_F(MecanumDriveOdometry3dTest, AccuracyFacingTrajectory) {
  frc::MecanumDriveKinematics kinematics{
      frc::Translation2d{1.0 * mp::m, 1.0 * mp::m},
      frc::Translation2d{1.0 * mp::m, -1.0 * mp::m},
      frc::Translation2d{-1.0 * mp::m, -1.0 * mp::m},
      frc::Translation2d{-1.0 * mp::m, 1.0 * mp::m}};

  frc::MecanumDriveWheelPositions wheelPositions;

  frc::MecanumDriveOdometry3d odometry{kinematics, frc::Rotation3d{},
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
        frc::Rotation3d{
            groundTruthState.pose.Rotation() +
            frc::Rotation2d{distribution(generator) * 0.05 * mp::rad}},
        wheelPositions);
    double error = mp::value(groundTruthState.pose.Translation().Distance(
        xhat.Translation().ToTranslation2d()));

    if (error > maxError) {
      maxError = error;
    }
    errorSum += error;

    t += dt;
  }

  EXPECT_LT(errorSum / double{trajectory.TotalTime() / dt}, 0.06);
  EXPECT_LT(maxError, 0.125);
}

TEST_F(MecanumDriveOdometry3dTest, AccuracyFacingXAxis) {
  frc::MecanumDriveKinematics kinematics{
      frc::Translation2d{1.0 * mp::m, 1.0 * mp::m},
      frc::Translation2d{1.0 * mp::m, -1.0 * mp::m},
      frc::Translation2d{-1.0 * mp::m, -1.0 * mp::m},
      frc::Translation2d{-1.0 * mp::m, 1.0 * mp::m}};

  frc::MecanumDriveWheelPositions wheelPositions;

  frc::MecanumDriveOdometry3d odometry{kinematics, frc::Rotation3d{},
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
        frc::Rotation3d{0.0 * mp::rad, 0.0 * mp::rad,
                        distribution(generator) * 0.05 * mp::rad},
        wheelPositions);
    double error = mp::value(groundTruthState.pose.Translation().Distance(
        xhat.Translation().ToTranslation2d()));

    if (error > maxError) {
      maxError = error;
    }
    errorSum += error;

    t += dt;
  }

  EXPECT_LT(errorSum / double{trajectory.TotalTime() / dt}, 0.06);
  EXPECT_LT(maxError, 0.125);
}
