// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <limits>
#include <numbers>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <wpi/print.h>

#include "frc/StateSpaceUtil.h"
#include "frc/estimator/DifferentialDrivePoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/units.h"

void testFollowTrajectory(
    const frc::DifferentialDriveKinematics& kinematics,
    frc::DifferentialDrivePoseEstimator& estimator,
    const frc::Trajectory& trajectory,
    std::function<frc::ChassisSpeeds(frc::Trajectory::State&)>
        chassisSpeedsGenerator,
    std::function<frc::Pose2d(frc::Trajectory::State&)>
        visionMeasurementGenerator,
    const frc::Pose2d& startingPose, const frc::Pose2d& endingPose,
    const mp::quantity<mp::s> dt, const mp::quantity<mp::s> kVisionUpdateRate,
    const mp::quantity<mp::s> kVisionUpdateDelay, const bool checkError,
    const bool debug) {
  mp::quantity<mp::m> leftDistance = 0.0 * mp::m;
  mp::quantity<mp::m> rightDistance = 0.0 * mp::m;

  estimator.ResetPosition(frc::Rotation2d{}, leftDistance, rightDistance,
                          startingPose);

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 1.0);

  mp::quantity<mp::s> t = 0.0 * mp::s;

  std::vector<std::pair<mp::quantity<mp::s>, frc::Pose2d>> visionPoses;
  std::vector<std::tuple<mp::quantity<mp::s>, mp::quantity<mp::s>, frc::Pose2d>>
      visionLog;

  double maxError = -std::numeric_limits<double>::max();
  double errorSum = 0;

  if (debug) {
    wpi::print(
        "time, est_x, est_y, est_theta, true_x, true_y, true_theta, left, "
        "right\n");
  }

  while (t < trajectory.TotalTime()) {
    frc::Trajectory::State groundTruthState = trajectory.Sample(t);

    // We are due for a new vision measurement if it's been `visionUpdateRate`
    // seconds since the last vision measurement
    if (visionPoses.empty() ||
        visionPoses.back().first + kVisionUpdateRate < t) {
      auto visionPose =
          visionMeasurementGenerator(groundTruthState) +
          frc::Transform2d{
              frc::Translation2d{distribution(generator) * 0.1 * mp::m,
                                 distribution(generator) * 0.1 * mp::m},
              frc::Rotation2d{distribution(generator) * 0.05 * mp::rad}};
      visionPoses.push_back({t, visionPose});
    }

    // We should apply the oldest vision measurement if it has been
    // `visionUpdateDelay` seconds since it was measured
    if (!visionPoses.empty() &&
        visionPoses.front().first + kVisionUpdateDelay < t) {
      auto visionEntry = visionPoses.front();
      estimator.AddVisionMeasurement(visionEntry.second, visionEntry.first);
      visionPoses.erase(visionPoses.begin());
      visionLog.push_back({t, visionEntry.first, visionEntry.second});
    }

    auto chassisSpeeds = chassisSpeedsGenerator(groundTruthState);

    auto wheelSpeeds = kinematics.ToWheelSpeeds(chassisSpeeds);

    leftDistance += wheelSpeeds.left * dt;
    rightDistance += wheelSpeeds.right * dt;

    auto xhat = estimator.UpdateWithTime(
        t,
        groundTruthState.pose.Rotation() +
            frc::Rotation2d{distribution(generator) * 0.05 * mp::rad} -
            trajectory.InitialPose().Rotation(),
        leftDistance, rightDistance);

    if (debug) {
      wpi::print("{}, {}, {}, {}, {}, {}, {}, {}, {}\n", mp::value(t),
                 mp::value(xhat.X()), mp::value(xhat.Y()),
                 mp::value(xhat.Rotation().Radians()),
                 mp::value(groundTruthState.pose.X()),
                 mp::value(groundTruthState.pose.Y()),
                 mp::value(groundTruthState.pose.Rotation().Radians()),
                 mp::value(leftDistance), mp::value(rightDistance));
    }

    double error = mp::value(
        groundTruthState.pose.Translation().Distance(xhat.Translation()));

    if (error > maxError) {
      maxError = error;
    }
    errorSum += error;

    t += dt;
  }

  if (debug) {
    wpi::print("apply_time, measured_time, vision_x, vision_y, vision_theta\n");

    mp::quantity<mp::s> apply_time;
    mp::quantity<mp::s> measure_time;
    frc::Pose2d vision_pose;
    for (auto record : visionLog) {
      std::tie(apply_time, measure_time, vision_pose) = record;
      wpi::print("{}, {}, {}, {}, {}\n", mp::value(apply_time),
                 mp::value(measure_time), mp::value(vision_pose.X()),
                 mp::value(vision_pose.Y()),
                 mp::value(vision_pose.Rotation().Radians()));
    }
  }

  EXPECT_NEAR(mp::value(endingPose.X()),
              mp::value(estimator.GetEstimatedPosition().X()), 0.08);
  EXPECT_NEAR(mp::value(endingPose.Y()),
              mp::value(estimator.GetEstimatedPosition().Y()), 0.08);
  EXPECT_NEAR(mp::value(endingPose.Rotation().Radians()),
              mp::value(estimator.GetEstimatedPosition().Rotation().Radians()),
              0.15);

  if (checkError) {
    // NOLINTNEXTLINE(bugprone-integer-division)
    EXPECT_LT(errorSum / (trajectory.TotalTime() / dt), 0.05);
    EXPECT_LT(maxError, 0.2);
  }
}

TEST(DifferentialDrivePoseEstimatorTest, Accuracy) {
  frc::DifferentialDriveKinematics kinematics{1.0 * mp::m};

  frc::DifferentialDrivePoseEstimator estimator{
      kinematics,    frc::Rotation2d{},  0.0 * mp::m,    0.0 * mp::m,
      frc::Pose2d{}, {0.02, 0.02, 0.01}, {0.1, 0.1, 0.1}};

  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      std::vector{frc::Pose2d{0.0 * mp::m, 0.0 * mp::m, 45.0 * mp::deg},
                  frc::Pose2d{3.0 * mp::m, 0.0 * mp::m, -90.0 * mp::deg},
                  frc::Pose2d{0.0 * mp::m, 0.0 * mp::m, 135.0 * mp::deg},
                  frc::Pose2d{-3.0 * mp::m, 0.0 * mp::m, -90.0 * mp::deg},
                  frc::Pose2d{0.0 * mp::m, 0.0 * mp::m, 45.0 * mp::deg}},
      frc::TrajectoryConfig(2.0 * mp::m / mp::s, 2.0 * mp::m / mp::s2));

  testFollowTrajectory(
      kinematics, estimator, trajectory,
      [&](frc::Trajectory::State& state) {
        return frc::ChassisSpeeds{state.velocity, 0.0 * mp::m / mp::s,
                                  state.velocity * state.curvature};
      },
      [&](frc::Trajectory::State& state) { return state.pose; },
      trajectory.InitialPose(),
      {0.0 * mp::m, 0.0 * mp::m, frc::Rotation2d{45.0 * mp::deg}},
      20.0 * mp::ms, 100.0 * mp::ms, 250.0 * mp::ms, true, false);
}

TEST(DifferentialDrivePoseEstimatorTest, BadInitialPose) {
  frc::DifferentialDriveKinematics kinematics{1.0 * mp::m};

  frc::DifferentialDrivePoseEstimator estimator{
      kinematics,    frc::Rotation2d{},  0.0 * mp::m,    0.0 * mp::m,
      frc::Pose2d{}, {0.02, 0.02, 0.01}, {0.1, 0.1, 0.1}};

  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      std::vector{frc::Pose2d{0.0 * mp::m, 0.0 * mp::m, 45.0 * mp::deg},
                  frc::Pose2d{3.0 * mp::m, 0.0 * mp::m, -90.0 * mp::deg},
                  frc::Pose2d{0.0 * mp::m, 0.0 * mp::m, 135.0 * mp::deg},
                  frc::Pose2d{-3.0 * mp::m, 0.0 * mp::m, -90.0 * mp::deg},
                  frc::Pose2d{0.0 * mp::m, 0.0 * mp::m, 45.0 * mp::deg}},
      frc::TrajectoryConfig(2.0 * mp::m / mp::s, 2.0 * mp::m / mp::s2));

  for (mp::quantity<mp::deg> offset_direction_degs = 0.0 * mp::deg;
       offset_direction_degs < 360.0 * mp::deg;
       offset_direction_degs += 45.0 * mp::deg) {
    for (mp::quantity<mp::deg> offset_heading_degs = 0.0 * mp::deg;
         offset_heading_degs < 360.0 * mp::deg;
         offset_heading_degs += 45.0 * mp::deg) {
      auto pose_offset = frc::Rotation2d{offset_direction_degs};
      auto heading_offset = frc::Rotation2d{offset_heading_degs};

      auto initial_pose =
          trajectory.InitialPose() +
          frc::Transform2d{frc::Translation2d{pose_offset.Cos() * 1.0 * mp::m,
                                              pose_offset.Sin() * 1.0 * mp::m},
                           heading_offset};

      testFollowTrajectory(
          kinematics, estimator, trajectory,
          [&](frc::Trajectory::State& state) {
            return frc::ChassisSpeeds{state.velocity, 0.0 * mp::m / mp::s,
                                      state.velocity * state.curvature};
          },
          [&](frc::Trajectory::State& state) { return state.pose; },
          initial_pose,
          {0.0 * mp::m, 0.0 * mp::m, frc::Rotation2d{45.0 * mp::deg}},
          20.0 * mp::ms, 100.0 * mp::ms, 250.0 * mp::ms, false, false);
    }
  }
}

TEST(DifferentialDrivePoseEstimatorTest, SimultaneousVisionMeasurements) {
  // This tests for multiple vision measurements applied at the same time.
  // The expected behavior is that all measurements affect the estimated pose.
  // The alternative result is that only one vision measurement affects the
  // outcome. If that were the case, after 1000 measurements, the estimated
  // pose would converge to that measurement.
  frc::DifferentialDriveKinematics kinematics{1.0 * mp::m};

  frc::DifferentialDrivePoseEstimator estimator{
      kinematics,
      frc::Rotation2d{},
      0.0 * mp::m,
      0.0 * mp::m,
      frc::Pose2d{1.0 * mp::m, 2.0 * mp::m, frc::Rotation2d{270.0 * mp::deg}},
      {0.02, 0.02, 0.01},
      {0.1, 0.1, 0.1}};

  estimator.UpdateWithTime(0.0 * mp::s, frc::Rotation2d{}, 0.0 * mp::m,
                           0.0 * mp::m);

  for (int i = 0; i < 1000; i++) {
    estimator.AddVisionMeasurement(
        frc::Pose2d{0.0 * mp::m, 0.0 * mp::m, frc::Rotation2d{0.0 * mp::deg}},
        0.0 * mp::s);
    estimator.AddVisionMeasurement(
        frc::Pose2d{3.0 * mp::m, 1.0 * mp::m, frc::Rotation2d{90.0 * mp::deg}},
        0.0 * mp::s);
    estimator.AddVisionMeasurement(
        frc::Pose2d{2.0 * mp::m, 4.0 * mp::m, frc::Rotation2d{180.0 * mp::deg}},
        0.0 * mp::s);
  }

  {
    auto dx = mp::abs(estimator.GetEstimatedPosition().X() - 0.0 * mp::m);
    auto dy = mp::abs(estimator.GetEstimatedPosition().Y() - 0.0 * mp::m);
    auto dtheta = mp::abs(
        estimator.GetEstimatedPosition().Rotation().Radians() - 0.0 * mp::deg);

    EXPECT_TRUE(dx > 0.08 * mp::m || dy > 0.08 * mp::m ||
                dtheta > 0.08 * mp::rad);
  }

  {
    auto dx = mp::abs(estimator.GetEstimatedPosition().X() - 3.0 * mp::m);
    auto dy = mp::abs(estimator.GetEstimatedPosition().Y() - 1.0 * mp::m);
    auto dtheta = mp::abs(
        estimator.GetEstimatedPosition().Rotation().Radians() - 90.0 * mp::deg);

    EXPECT_TRUE(dx > 0.08 * mp::m || dy > 0.08 * mp::m ||
                dtheta > 0.08 * mp::rad);
  }

  {
    auto dx = mp::abs(estimator.GetEstimatedPosition().X() - 2.0 * mp::m);
    auto dy = mp::abs(estimator.GetEstimatedPosition().Y() - 4.0 * mp::m);
    auto dtheta =
        mp::abs(estimator.GetEstimatedPosition().Rotation().Radians() -
                180.0 * mp::deg);

    EXPECT_TRUE(dx > 0.08 * mp::m || dy > 0.08 * mp::m ||
                dtheta > 0.08 * mp::rad);
  }
}

TEST(DifferentialDrivePoseEstimatorTest, TestDiscardStaleVisionMeasurements) {
  frc::DifferentialDriveKinematics kinematics{1.0 * mp::m};

  frc::DifferentialDrivePoseEstimator estimator{
      kinematics,    frc::Rotation2d{}, 0.0 * mp::m,       0.0 * mp::m,
      frc::Pose2d{}, {0.1, 0.1, 0.1},   {0.45, 0.45, 0.45}};

  // Add enough measurements to fill up the buffer
  for (auto time = 0.0 * mp::s; time < 4.0 * mp::s; time += 20.0 * mp::ms) {
    estimator.UpdateWithTime(time, frc::Rotation2d{}, 0.0 * mp::m, 0.0 * mp::m);
  }

  auto odometryPose = estimator.GetEstimatedPosition();

  // Apply a vision measurement from 3 seconds ago
  estimator.AddVisionMeasurement(
      frc::Pose2d{frc::Translation2d{10.0 * mp::m, 10.0 * mp::m},
                  frc::Rotation2d{0.1 * mp::rad}},
      1.0 * mp::s, {0.1, 0.1, 0.1});

  EXPECT_NEAR(mp::value(odometryPose.X()),
              mp::value(estimator.GetEstimatedPosition().X()), 1e-6);
  EXPECT_NEAR(mp::value(odometryPose.Y()),
              mp::value(estimator.GetEstimatedPosition().Y()), 1e-6);
  EXPECT_NEAR(mp::value(odometryPose.Rotation().Radians()),
              mp::value(estimator.GetEstimatedPosition().Rotation().Radians()),
              1e-6);
}

TEST(DifferentialDrivePoseEstimatorTest, TestSampleAt) {
  frc::DifferentialDriveKinematics kinematics{1.0 * mp::m};
  frc::DifferentialDrivePoseEstimator estimator{
      kinematics,    frc::Rotation2d{}, 0.0 * mp::m,    0.0 * mp::m,
      frc::Pose2d{}, {1.0, 1.0, 1.0},   {1.0, 1.0, 1.0}};

  // Returns empty when null
  EXPECT_EQ(std::nullopt, estimator.SampleAt(1.0 * mp::s));

  // Add odometry measurements, but don't fill up the buffer
  // Add a tiny tolerance for the upper bound because of floating point rounding
  // error
  for (double time = 1; time <= 2 + 1e-9; time += 0.02) {
    estimator.UpdateWithTime(time * mp::s, frc::Rotation2d{}, time * mp::m,
                             time * mp::m);
  }

  // Sample at an added time
  EXPECT_EQ(
      std::optional(frc::Pose2d{1.02 * mp::m, 0.0 * mp::m, frc::Rotation2d{}}),
      estimator.SampleAt(1.02 * mp::s));
  // Sample between updates (test interpolation)
  EXPECT_EQ(
      std::optional(frc::Pose2d{1.01 * mp::m, 0.0 * mp::m, frc::Rotation2d{}}),
      estimator.SampleAt(1.01 * mp::s));
  // Sampling before the oldest value returns the oldest value
  EXPECT_EQ(
      std::optional(frc::Pose2d{1.0 * mp::m, 0.0 * mp::m, frc::Rotation2d{}}),
      estimator.SampleAt(0.5 * mp::s));
  // Sampling after the newest value returns the newest value
  EXPECT_EQ(
      std::optional(frc::Pose2d{2.0 * mp::m, 0.0 * mp::m, frc::Rotation2d{}}),
      estimator.SampleAt(2.5 * mp::s));

  // Add a vision measurement after the odometry measurements (while keeping all
  // of the old odometry measurements)
  estimator.AddVisionMeasurement(
      frc::Pose2d{2.0 * mp::m, 0.0 * mp::m, frc::Rotation2d{1.0 * mp::rad}},
      2.2 * mp::s);

  // Make sure nothing changed (except the newest value)
  EXPECT_EQ(
      std::optional(frc::Pose2d{1.02 * mp::m, 0.0 * mp::m, frc::Rotation2d{}}),
      estimator.SampleAt(1.02 * mp::s));
  EXPECT_EQ(
      std::optional(frc::Pose2d{1.01 * mp::m, 0.0 * mp::m, frc::Rotation2d{}}),
      estimator.SampleAt(1.01 * mp::s));
  EXPECT_EQ(
      std::optional(frc::Pose2d{1.0 * mp::m, 0.0 * mp::m, frc::Rotation2d{}}),
      estimator.SampleAt(0.5 * mp::s));

  // Add a vision measurement before the odometry measurements that's still in
  // the buffer
  estimator.AddVisionMeasurement(
      frc::Pose2d{1.0 * mp::m, 0.2 * mp::m, frc::Rotation2d{}}, 0.9 * mp::s);

  // Everything should be the same except Y is 0.1 (halfway between 0 and 0.2)
  EXPECT_EQ(
      std::optional(frc::Pose2d{1.02 * mp::m, 0.1 * mp::m, frc::Rotation2d{}}),
      estimator.SampleAt(1.02 * mp::s));
  EXPECT_EQ(
      std::optional(frc::Pose2d{1.01 * mp::m, 0.1 * mp::m, frc::Rotation2d{}}),
      estimator.SampleAt(1.01 * mp::s));
  EXPECT_EQ(
      std::optional(frc::Pose2d{1.0 * mp::m, 0.1 * mp::m, frc::Rotation2d{}}),
      estimator.SampleAt(0.5 * mp::s));
  EXPECT_EQ(
      std::optional(frc::Pose2d{2.0 * mp::m, 0.1 * mp::m, frc::Rotation2d{}}),
      estimator.SampleAt(2.5 * mp::s));
}

TEST(DifferentialDrivePoseEstimatorTest, TestReset) {
  frc::DifferentialDriveKinematics kinematics{1.0 * mp::m};
  frc::DifferentialDrivePoseEstimator estimator{
      kinematics,
      frc::Rotation2d{},
      0.0 * mp::m,
      0.0 * mp::m,
      frc::Pose2d{-1.0 * mp::m, -1.0 * mp::m, frc::Rotation2d{1.0 * mp::rad}},
      {1.0, 1.0, 1.0},
      {1.0, 1.0, 1.0}};

  // Test initial pose
  EXPECT_DOUBLE_EQ(-1, mp::value(estimator.GetEstimatedPosition().X()));
  EXPECT_DOUBLE_EQ(-1, mp::value(estimator.GetEstimatedPosition().Y()));
  EXPECT_DOUBLE_EQ(
      1, mp::value(estimator.GetEstimatedPosition().Rotation().Radians()));

  // Test reset position
  estimator.ResetPosition(
      frc::Rotation2d{}, 1.0 * mp::m, 1.0 * mp::m,
      frc::Pose2d{1.0 * mp::m, 0.0 * mp::m, frc::Rotation2d{}});

  EXPECT_DOUBLE_EQ(1, mp::value(estimator.GetEstimatedPosition().X()));
  EXPECT_DOUBLE_EQ(0, mp::value(estimator.GetEstimatedPosition().Y()));
  EXPECT_DOUBLE_EQ(
      0, mp::value(estimator.GetEstimatedPosition().Rotation().Radians()));

  // Test orientation and wheel positions
  estimator.Update(frc::Rotation2d{}, 2.0 * mp::m, 2.0 * mp::m);

  EXPECT_DOUBLE_EQ(2, mp::value(estimator.GetEstimatedPosition().X()));
  EXPECT_DOUBLE_EQ(0, mp::value(estimator.GetEstimatedPosition().Y()));
  EXPECT_DOUBLE_EQ(
      0, mp::value(estimator.GetEstimatedPosition().Rotation().Radians()));

  // Test reset rotation
  estimator.ResetRotation(frc::Rotation2d{90.0 * mp::deg});

  EXPECT_DOUBLE_EQ(2, mp::value(estimator.GetEstimatedPosition().X()));
  EXPECT_DOUBLE_EQ(0, mp::value(estimator.GetEstimatedPosition().Y()));
  EXPECT_DOUBLE_EQ(
      std::numbers::pi / 2,
      mp::value(estimator.GetEstimatedPosition().Rotation().Radians()));

  // Test orientation
  estimator.Update(frc::Rotation2d{}, 3.0 * mp::m, 3.0 * mp::m);

  EXPECT_DOUBLE_EQ(2, mp::value(estimator.GetEstimatedPosition().X()));
  EXPECT_DOUBLE_EQ(1, mp::value(estimator.GetEstimatedPosition().Y()));
  EXPECT_DOUBLE_EQ(
      std::numbers::pi / 2,
      mp::value(estimator.GetEstimatedPosition().Rotation().Radians()));

  // Test reset translation
  estimator.ResetTranslation(frc::Translation2d{-1.0 * mp::m, -1.0 * mp::m});

  EXPECT_DOUBLE_EQ(-1, mp::value(estimator.GetEstimatedPosition().X()));
  EXPECT_DOUBLE_EQ(-1, mp::value(estimator.GetEstimatedPosition().Y()));
  EXPECT_DOUBLE_EQ(
      std::numbers::pi / 2,
      mp::value(estimator.GetEstimatedPosition().Rotation().Radians()));

  // Test reset pose
  estimator.ResetPose(frc::Pose2d{});

  EXPECT_DOUBLE_EQ(0, mp::value(estimator.GetEstimatedPosition().X()));
  EXPECT_DOUBLE_EQ(0, mp::value(estimator.GetEstimatedPosition().Y()));
  EXPECT_DOUBLE_EQ(
      0, mp::value(estimator.GetEstimatedPosition().Rotation().Radians()));
}
