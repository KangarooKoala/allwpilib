// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "frc/trajectory/constraint/CentripetalAccelerationConstraint.h"
#include "frc/trajectory/constraint/TrajectoryConstraint.h"
#include "frc/units-usc.h"
#include "frc/units.h"
#include "trajectory/TestTrajectory.h"

using namespace frc;

TEST(CentripetalAccelerationConstraintTest, Constraint) {
  const auto maxCentripetalAcceleration = 7.0 * mp::ft / mp::s2;

  auto config = TrajectoryConfig(12.0 * mp::ft / mp::s, 12.0 * mp::ft / mp::s2);
  config.AddConstraint(
      CentripetalAccelerationConstraint(maxCentripetalAcceleration));

  auto trajectory = TestTrajectory::GetTrajectory(config);

  mp::quantity<mp::s> time = 0.0 * mp::s;
  mp::quantity<mp::s> dt = 20.0 * mp::ms;
  mp::quantity<mp::s> duration = trajectory.TotalTime();

  while (time < duration) {
    const Trajectory::State point = trajectory.Sample(time);
    time += dt;

    auto centripetalAcceleration =
        mp::pow<2>(point.velocity) * point.curvature / (1.0 * mp::rad);

    EXPECT_TRUE(centripetalAcceleration <
                maxCentripetalAcceleration + 0.05 * mp::m / mp::s2);
  }
}
