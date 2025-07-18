// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <vector>

#include <gtest/gtest.h>

#include "frc/trajectory/Trajectory.h"
#include "frc/trajectory/TrajectoryConfig.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/units.h"

void TestSameShapedTrajectory(std::vector<frc::Trajectory::State> statesA,
                              std::vector<frc::Trajectory::State> statesB) {
  for (unsigned int i = 0; i < statesA.size() - 1; i++) {
    auto a1 = statesA[i].pose;
    auto a2 = statesA[i + 1].pose;

    auto b1 = statesB[i].pose;
    auto b2 = statesB[i + 1].pose;

    auto a = a2.RelativeTo(a1);
    auto b = b2.RelativeTo(b1);

    EXPECT_NEAR(mp::value(a.X()), mp::value(b.X()), 1E-9);
    EXPECT_NEAR(mp::value(a.Y()), mp::value(b.Y()), 1E-9);
    EXPECT_NEAR(mp::value(a.Rotation().Radians()),
                mp::value(b.Rotation().Radians()), 1E-9);
  }
}

TEST(TrajectoryTransformsTest, TransformBy) {
  frc::TrajectoryConfig config{3.0 * mp::m / mp::s, 3.0 * mp::m / mp::s2};
  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      frc::Pose2d{}, {}, frc::Pose2d{1.0 * mp::m, 1.0 * mp::m, 90.0 * mp::deg},
      config);

  auto transformedTrajectory =
      trajectory.TransformBy({{1.0 * mp::m, 2.0 * mp::m}, 30.0 * mp::deg});

  auto firstPose = transformedTrajectory.Sample(0.0 * mp::s).pose;

  EXPECT_NEAR(mp::value(firstPose.X()), 1.0, 1E-9);
  EXPECT_NEAR(mp::value(firstPose.Y()), 2.0, 1E-9);
  EXPECT_NEAR(mp::value(firstPose.Rotation().Degrees()), 30.0, 1E-9);

  TestSameShapedTrajectory(trajectory.States(), transformedTrajectory.States());
}

TEST(TrajectoryTransformsTest, RelativeTo) {
  frc::TrajectoryConfig config{3.0 * mp::m / mp::s, 3.0 * mp::m / mp::s2};
  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      frc::Pose2d{1.0 * mp::m, 2.0 * mp::m, 30.0 * mp::deg}, {},
      frc::Pose2d{5.0 * mp::m, 7.0 * mp::m, 90.0 * mp::deg}, config);

  auto transformedTrajectory =
      trajectory.RelativeTo({1.0 * mp::m, 2.0 * mp::m, 30.0 * mp::deg});

  auto firstPose = transformedTrajectory.Sample(0.0 * mp::s).pose;

  EXPECT_NEAR(mp::value(firstPose.X()), 0, 1E-9);
  EXPECT_NEAR(mp::value(firstPose.Y()), 0, 1E-9);
  EXPECT_NEAR(mp::value(firstPose.Rotation().Degrees()), 0, 1E-9);

  TestSameShapedTrajectory(trajectory.States(), transformedTrajectory.States());
}
