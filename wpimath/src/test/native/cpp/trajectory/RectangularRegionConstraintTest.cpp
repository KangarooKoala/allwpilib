// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/trajectory/constraint/MaxVelocityConstraint.h"
#include "frc/trajectory/constraint/RectangularRegionConstraint.h"
#include "frc/units-usc.h"
#include "frc/units.h"
#include "trajectory/TestTrajectory.h"

using namespace frc;

TEST(RectangularRegionConstraintTest, Constraint) {
  constexpr auto maxVelocity = 2.0 * mp::ft / mp::s;
  constexpr frc::Rectangle2d rectangle{{1.0 * mp::ft, 1.0 * mp::ft},
                                       {5.0 * mp::ft, 27.0 * mp::ft}};

  auto config = TrajectoryConfig(13.0 * mp::ft / mp::s, 13.0 * mp::ft / mp::s2);
  config.AddConstraint(RectangularRegionConstraint{
      rectangle, MaxVelocityConstraint{maxVelocity}});
  auto trajectory = TestTrajectory::GetTrajectory(config);

  bool exceededConstraintOutsideRegion = false;
  for (auto& point : trajectory.States()) {
    if (rectangle.Contains(point.pose.Translation())) {
      EXPECT_TRUE(mp::abs(point.velocity) < maxVelocity + 0.05 * mp::m / mp::s);
    } else if (mp::abs(point.velocity) >= maxVelocity + 0.05 * mp::m / mp::s) {
      exceededConstraintOutsideRegion = true;
    }
  }
  EXPECT_TRUE(exceededConstraintOutsideRegion);
}
