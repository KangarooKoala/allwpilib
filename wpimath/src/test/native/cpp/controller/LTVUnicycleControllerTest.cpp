// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/MathUtil.h"
#include "frc/controller/LTVUnicycleController.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/units.h"

#define EXPECT_NEAR_UNITS(val1, val2, eps) EXPECT_LE(mp::abs(val1 - val2), eps)

static constexpr mp::quantity<mp::m> kTolerance = 1.0 / 12.0 * mp::m;
static constexpr mp::quantity<mp::rad> kAngularTolerance = 2.0 * mp::deg;

TEST(LTVUnicycleControllerTest, ReachesReference) {
  constexpr mp::quantity<mp::s> kDt = 20.0 * mp::ms;

  frc::LTVUnicycleController controller{{0.0625, 0.125, 2.5}, {4.0, 4.0}, kDt};
  frc::Pose2d robotPose{2.7 * mp::m, 23.0 * mp::m, 0.0 * mp::deg};

  auto waypoints =
      std::vector{frc::Pose2d{2.75 * mp::m, 22.521 * mp::m, 0.0 * mp::rad},
                  frc::Pose2d{24.73 * mp::m, 19.68 * mp::m, 5.846 * mp::rad}};
  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      waypoints, {8.8 * mp::m / mp::s, 0.1 * mp::m / mp::s2});

  auto totalTime = trajectory.TotalTime();
  for (size_t i = 0; i < mp::value(totalTime / kDt); ++i) {
    auto state = trajectory.Sample(kDt * i);
    auto [vx, vy, omega] = controller.Calculate(robotPose, state);
    static_cast<void>(vy);

    robotPose =
        robotPose + frc::Twist2d{vx * kDt, 0.0 * mp::m, omega * kDt}.Exp();
  }

  auto& endPose = trajectory.States().back().pose;
  EXPECT_NEAR_UNITS(endPose.X(), robotPose.X(), kTolerance);
  EXPECT_NEAR_UNITS(endPose.Y(), robotPose.Y(), kTolerance);
  EXPECT_NEAR_UNITS(frc::AngleModulus(endPose.Rotation().Radians() -
                                      robotPose.Rotation().Radians()),
                    0.0 * mp::rad, kAngularTolerance);
}
