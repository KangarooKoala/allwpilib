// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/math/trajectory/TrajectoryGenerator.hpp"

#include <vector>

#include <gtest/gtest.h>

#include "wpi/math/trajectory/TestTrajectory.hpp"
#include "wpi/math/trajectory/Trajectory.hpp"
#include "wpi/math/trajectory/constraint/CentripetalAccelerationConstraint.hpp"
#include "wpi/math/trajectory/constraint/TrajectoryConstraint.hpp"

using namespace wpi::math;

TEST(TrajectoryGenerationTest, ObeysConstraints) {
  TrajectoryConfig config{12_fps, 12_fps2};
  auto trajectory = TestTrajectory::GetTrajectory(config);

  wpi::units::seconds<> time = 0_s;
  wpi::units::seconds<> dt = 20_ms;
  wpi::units::seconds<> duration = trajectory.TotalTime();

  while (time < duration) {
    const Trajectory::State point = trajectory.Sample(time);
    time += dt;

    EXPECT_TRUE(wpi::units::abs(point.velocity) <= 12_fps + 0.01_fps);
    EXPECT_TRUE(wpi::units::abs(point.acceleration) <=
                12_fps2 + 0.01_fps2);
  }
}

TEST(TrajectoryGenertionTest, ReturnsEmptyOnMalformed) {
  const auto t = TrajectoryGenerator::GenerateTrajectory(
      std::vector<Pose2d>{Pose2d{0_m, 0_m, 0_deg}, Pose2d{1_m, 0_m, 180_deg}},
      TrajectoryConfig(12_fps, 12_fps2));

  ASSERT_EQ(t.States().size(), 1u);
  ASSERT_EQ(t.TotalTime(), 0_s);
}

TEST(TrajectoryGenerationTest, CurvatureOptimization) {
  auto t = TrajectoryGenerator::GenerateTrajectory(
      {{1_m, 0_m, 90_deg},
       {0_m, 1_m, 180_deg},
       {-1_m, 0_m, 270_deg},
       {0_m, -1_m, 0_deg},
       {1_m, 0_m, 90_deg}},
      TrajectoryConfig{12_fps, 12_fps2});

  for (size_t i = 1; i < t.States().size() - 1; ++i) {
    EXPECT_NE(0, t.States()[i].curvature.value());
  }
}
