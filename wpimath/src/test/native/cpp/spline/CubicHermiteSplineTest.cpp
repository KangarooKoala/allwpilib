// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <chrono>
#include <vector>

#include <gtest/gtest.h>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/spline/QuinticHermiteSpline.h"
#include "frc/spline/SplineHelper.h"
#include "frc/spline/SplineParameterizer.h"
#include "frc/units.h"

using namespace frc;

namespace frc {
class CubicHermiteSplineTest : public ::testing::Test {
 protected:
  static void Run(const Pose2d& a, const std::vector<Translation2d>& waypoints,
                  const Pose2d& b) {
    // Generate and parameterize the spline.

    const auto [startCV, endCV] =
        SplineHelper::CubicControlVectorsFromWaypoints(a, waypoints, b);

    const auto splines =
        SplineHelper::CubicSplinesFromControlVectors(startCV, waypoints, endCV);
    std::vector<Spline<3>::PoseWithCurvature> poses;

    poses.push_back(splines[0].GetPoint(0.0).value());

    for (auto&& spline : splines) {
      auto x = SplineParameterizer::Parameterize(spline);
      poses.insert(std::end(poses), std::begin(x) + 1, std::end(x));
    }

    for (unsigned int i = 0; i < poses.size() - 1; i++) {
      auto& p0 = poses[i];
      auto& p1 = poses[i + 1];

      // Make sure the twist is under the tolerance defined by the Spline class.
      auto twist = (p1.first - p0.first).Log();
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

    // Check interior waypoints
    bool interiorsGood = true;
    for (auto& waypoint : waypoints) {
      bool found = false;
      for (auto& state : poses) {
        if (std::abs(mp::value(waypoint.Distance(state.first.Translation()))) <
            1E-9) {
          found = true;
        }
      }
      interiorsGood &= found;
    }

    EXPECT_TRUE(interiorsGood);

    // Check last point.
    EXPECT_NEAR(mp::value(poses.back().first.X()), mp::value(b.X()), 1E-9);
    EXPECT_NEAR(mp::value(poses.back().first.Y()), mp::value(b.Y()), 1E-9);
    EXPECT_NEAR(mp::value(poses.back().first.Rotation().Radians()),
                mp::value(b.Rotation().Radians()), 1E-9);
  }
};
}  // namespace frc

TEST_F(CubicHermiteSplineTest, StraightLine) {
  Run(Pose2d{}, std::vector<Translation2d>(),
      Pose2d{3.0 * mp::m, 0.0 * mp::m, 0.0 * mp::deg});
}

TEST_F(CubicHermiteSplineTest, SCurve) {
  Pose2d start{0.0 * mp::m, 0.0 * mp::m, 90.0 * mp::deg};
  std::vector<Translation2d> waypoints{
      Translation2d{1.0 * mp::m, 1.0 * mp::m},
      Translation2d{2.0 * mp::m, -1.0 * mp::m}};
  Pose2d end{3.0 * mp::m, 0.0 * mp::m, 90.0 * mp::deg};
  Run(start, waypoints, end);
}

TEST_F(CubicHermiteSplineTest, OneInterior) {
  Pose2d start{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::rad};
  std::vector<Translation2d> waypoints{Translation2d{2.0 * mp::m, 0.0 * mp::m}};
  Pose2d end{4.0 * mp::m, 0.0 * mp::m, 0.0 * mp::rad};
  Run(start, waypoints, end);
}

TEST_F(CubicHermiteSplineTest, ThrowsOnMalformed) {
  EXPECT_THROW(Run(Pose2d{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::deg},
                   std::vector<Translation2d>{},
                   Pose2d{1.0 * mp::m, 0.0 * mp::m, 180.0 * mp::deg}),
               SplineParameterizer::MalformedSplineException);
  EXPECT_THROW(Run(Pose2d{10.0 * mp::m, 10.0 * mp::m, 90.0 * mp::deg},
                   std::vector<Translation2d>{},
                   Pose2d{10.0 * mp::m, 11.0 * mp::m, -90.0 * mp::deg}),
               SplineParameterizer::MalformedSplineException);
}
