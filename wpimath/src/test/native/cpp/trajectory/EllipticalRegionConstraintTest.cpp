// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/trajectory/constraint/EllipticalRegionConstraint.h"
#include "frc/trajectory/constraint/MaxVelocityConstraint.h"
#include "frc/units-usc.h"
#include "frc/units.h"
#include "trajectory/TestTrajectory.h"

using namespace frc;

TEST(EllipticalRegionConstraintTest, Constraint) {
  constexpr auto maxVelocity = 2.0 * mp::ft / mp::s;
  constexpr frc::Ellipse2d ellipse{
      {5.0 * mp::ft, 2.5 * mp::ft, 180.0 * mp::deg},
      5.0 * mp::ft,
      2.5 * mp::ft};

  auto config = TrajectoryConfig(13.0 * mp::ft / mp::s, 13.0 * mp::ft / mp::s2);
  config.AddConstraint(
      EllipticalRegionConstraint{ellipse, MaxVelocityConstraint{maxVelocity}});
  auto trajectory = TestTrajectory::GetTrajectory(config);

  bool exceededConstraintOutsideRegion = false;
  for (auto& point : trajectory.States()) {
    if (ellipse.Contains(point.pose.Translation())) {
      EXPECT_TRUE(mp::abs(point.velocity) < maxVelocity + 0.05 * mp::m / mp::s);
    } else if (mp::abs(point.velocity) >= maxVelocity + 0.05 * mp::m / mp::s) {
      exceededConstraintOutsideRegion = true;
    }
  }
  EXPECT_TRUE(exceededConstraintOutsideRegion);
}
