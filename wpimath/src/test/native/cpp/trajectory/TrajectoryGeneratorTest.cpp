// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <vector>

#include <gtest/gtest.h>

#include "frc/trajectory/Trajectory.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/trajectory/constraint/CentripetalAccelerationConstraint.h"
#include "frc/trajectory/constraint/TrajectoryConstraint.h"
#include "frc/units-usc.h"
#include "frc/units.h"
#include "trajectory/TestTrajectory.h"

using namespace frc;

TEST(TrajectoryGenerationTest, ObeysConstraints) {
  TrajectoryConfig config{12.0 * mp::ft / mp::s, 12.0 * mp::ft / mp::s2};
  auto trajectory = TestTrajectory::GetTrajectory(config);

  mp::quantity<mp::s> time = 0.0 * mp::s;
  mp::quantity<mp::s> dt = 20.0 * mp::ms;
  mp::quantity<mp::s> duration = trajectory.TotalTime();

  while (time < duration) {
    const Trajectory::State point = trajectory.Sample(time);
    time += dt;

    EXPECT_TRUE(mp::abs(point.velocity) <=
                12.0 * mp::ft / mp::s + 0.01 * mp::ft / mp::s);
    EXPECT_TRUE(mp::abs(point.acceleration) <=
                12.0 * mp::ft / mp::s2 + 0.01 * mp::ft / mp::s2);
  }
}

TEST(TrajectoryGenertionTest, ReturnsEmptyOnMalformed) {
  const auto t = TrajectoryGenerator::GenerateTrajectory(
      std::vector<Pose2d>{Pose2d{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::deg},
                          Pose2d{1.0 * mp::m, 0.0 * mp::m, 180.0 * mp::deg}},
      TrajectoryConfig(12.0 * mp::ft / mp::s, 12.0 * mp::ft / mp::s2));

  ASSERT_EQ(t.States().size(), 1u);
  ASSERT_EQ(t.TotalTime(), 0.0 * mp::s);
}

TEST(TrajectoryGenerationTest, CurvatureOptimization) {
  auto t = TrajectoryGenerator::GenerateTrajectory(
      {{1.0 * mp::m, 0.0 * mp::m, 90.0 * mp::deg},
       {0.0 * mp::m, 1.0 * mp::m, 180.0 * mp::deg},
       {-1.0 * mp::m, 0.0 * mp::m, 270.0 * mp::deg},
       {0.0 * mp::m, -1.0 * mp::m, 0.0 * mp::deg},
       {1.0 * mp::m, 0.0 * mp::m, 90.0 * mp::deg}},
      TrajectoryConfig{12.0 * mp::ft / mp::s, 12.0 * mp::ft / mp::s2});

  for (size_t i = 1; i < t.States().size() - 1; ++i) {
    EXPECT_NE(0, mp::value(t.States()[i].curvature));
  }
}
