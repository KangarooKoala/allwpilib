// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <chrono>

#include <gtest/gtest.h>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/spline/QuinticHermiteSpline.h"
#include "frc/spline/SplineHelper.h"
#include "frc/spline/SplineParameterizer.h"
#include "frc/units.h"

using namespace frc;

namespace frc {
class QuinticHermiteSplineTest : public ::testing::Test {
 protected:
  static void Run(const Pose2d& a, const Pose2d& b) {
    // Generate and parameterize the spline.
    const auto spline = SplineHelper::QuinticSplinesFromWaypoints({a, b})[0];
    const auto poses = SplineParameterizer::Parameterize(spline);

    for (unsigned int i = 0; i < poses.size() - 1; i++) {
      auto& p0 = poses[i];
      auto& p1 = poses[i + 1];

      // Make sure the twist is under the tolerance defined by the Spline class.
      auto twist = p0.first.Log(p1.first);
      EXPECT_LT(std::abs(mp::value(twist.dx)),
                mp::value(SplineParameterizer::kMaxDx));
      EXPECT_LT(std::abs(mp::value(twist.dy)),
                mp::value(SplineParameterizer::kMaxDy));
      EXPECT_LT(std::abs(mp::value(twist.dtheta)),
                mp::value(SplineParameterizer::kMaxDtheta));
    }

    // Check first point.
    EXPECT_NEAR(mp::value(poses.front().first.X()), mp::value(a.X()), 1E-9);
    EXPECT_NEAR(mp::value(poses.front().first.Y()), mp::value(a.Y()), 1E-9);
    EXPECT_NEAR(mp::value(poses.front().first.Rotation().Radians()),
                mp::value(a.Rotation().Radians()), 1E-9);

    // Check last point.
    EXPECT_NEAR(mp::value(poses.back().first.X()), mp::value(b.X()), 1E-9);
    EXPECT_NEAR(mp::value(poses.back().first.Y()), mp::value(b.Y()), 1E-9);
    EXPECT_NEAR(mp::value(poses.back().first.Rotation().Radians()),
                mp::value(b.Rotation().Radians()), 1E-9);
  }
};
}  // namespace frc

TEST_F(QuinticHermiteSplineTest, StraightLine) {
  Run(Pose2d{}, Pose2d{3.0 * mp::m, 0.0 * mp::m, 0.0 * mp::deg});
}

TEST_F(QuinticHermiteSplineTest, SimpleSCurve) {
  Run(Pose2d{}, Pose2d{1.0 * mp::m, 1.0 * mp::m, 0.0 * mp::deg});
}

TEST_F(QuinticHermiteSplineTest, SquigglyCurve) {
  Run(Pose2d{0.0 * mp::m, 0.0 * mp::m, 90.0 * mp::deg},
      Pose2d{-1.0 * mp::m, 0.0 * mp::m, 90.0 * mp::deg});
}

TEST_F(QuinticHermiteSplineTest, ThrowsOnMalformed) {
  EXPECT_THROW(Run(Pose2d{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::deg},
                   Pose2d{1.0 * mp::m, 0.0 * mp::m, 180.0 * mp::deg}),
               SplineParameterizer::MalformedSplineException);
  EXPECT_THROW(Run(Pose2d{10.0 * mp::m, 10.0 * mp::m, 90.0 * mp::deg},
                   Pose2d{10.0 * mp::m, 11.0 * mp::m, -90.0 * mp::deg}),
               SplineParameterizer::MalformedSplineException);
}
