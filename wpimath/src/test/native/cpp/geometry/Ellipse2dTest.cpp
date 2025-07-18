// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/geometry/Ellipse2d.h"
#include "frc/units.h"

TEST(Ellipse2dTest, FocalPoints) {
  constexpr frc::Pose2d center{1.0 * mp::m, 2.0 * mp::m, 0.0 * mp::deg};
  constexpr frc::Ellipse2d ellipse{center, 5.0 * mp::m, 4.0 * mp::m};

  const auto [a, b] = ellipse.FocalPoints();

  EXPECT_EQ(frc::Translation2d(-2.0 * mp::m, 2.0 * mp::m), a);
  EXPECT_EQ(frc::Translation2d(4.0 * mp::m, 2.0 * mp::m), b);
}

TEST(Ellipse2dTest, Intersects) {
  constexpr frc::Pose2d center{1.0 * mp::m, 2.0 * mp::m, 0.0 * mp::deg};
  constexpr frc::Ellipse2d ellipse{center, 2.0 * mp::m, 1.0 * mp::m};

  constexpr frc::Translation2d pointA{1.0 * mp::m, 3.0 * mp::m};
  constexpr frc::Translation2d pointB{0.0 * mp::m, 3.0 * mp::m};

  EXPECT_TRUE(ellipse.Intersects(pointA));
  EXPECT_FALSE(ellipse.Intersects(pointB));
}

TEST(Ellipse2dTest, Contains) {
  constexpr frc::Pose2d center{-1.0 * mp::m, -2.0 * mp::m, 45.0 * mp::deg};
  constexpr frc::Ellipse2d ellipse{center, 2.0 * mp::m, 1.0 * mp::m};

  constexpr frc::Translation2d pointA{0.0 * mp::m, -1.0 * mp::m};
  constexpr frc::Translation2d pointB{0.5 * mp::m, -2.0 * mp::m};

  EXPECT_TRUE(ellipse.Contains(pointA));
  EXPECT_FALSE(ellipse.Contains(pointB));
}

TEST(Ellipse2dTest, Distance) {
  constexpr double kEpsilon = 1E-9;

  constexpr frc::Pose2d center{1.0 * mp::m, 2.0 * mp::m, 270.0 * mp::deg};
  constexpr frc::Ellipse2d ellipse{center, 1.0 * mp::m, 2.0 * mp::m};

  constexpr frc::Translation2d point1{2.5 * mp::m, 2.0 * mp::m};
  EXPECT_NEAR(0, mp::value(ellipse.Distance(point1)), kEpsilon);

  constexpr frc::Translation2d point2{1.0 * mp::m, 2.0 * mp::m};
  EXPECT_NEAR(0, mp::value(ellipse.Distance(point2)), kEpsilon);

  constexpr frc::Translation2d point3{1.0 * mp::m, 1.0 * mp::m};
  EXPECT_NEAR(0, mp::value(ellipse.Distance(point3)), kEpsilon);

  constexpr frc::Translation2d point4{-1.0 * mp::m, 2.5 * mp::m};
  EXPECT_NEAR(0.19210128384806818, mp::value(ellipse.Distance(point4)),
              kEpsilon);
}

TEST(Ellipse2dTest, Nearest) {
  constexpr double kEpsilon = 1E-9;

  constexpr frc::Pose2d center{1.0 * mp::m, 2.0 * mp::m, 270.0 * mp::deg};
  constexpr frc::Ellipse2d ellipse{center, 1.0 * mp::m, 2.0 * mp::m};

  constexpr frc::Translation2d point1{2.5 * mp::m, 2.0 * mp::m};
  auto nearestPoint1 = ellipse.Nearest(point1);
  EXPECT_NEAR(2.5, mp::value(nearestPoint1.X()), kEpsilon);
  EXPECT_NEAR(2.0, mp::value(nearestPoint1.Y()), kEpsilon);

  constexpr frc::Translation2d point2{1.0 * mp::m, 2.0 * mp::m};
  auto nearestPoint2 = ellipse.Nearest(point2);
  EXPECT_NEAR(1.0, mp::value(nearestPoint2.X()), kEpsilon);
  EXPECT_NEAR(2.0, mp::value(nearestPoint2.Y()), kEpsilon);

  constexpr frc::Translation2d point3{1.0 * mp::m, 1.0 * mp::m};
  auto nearestPoint3 = ellipse.Nearest(point3);
  EXPECT_NEAR(1.0, mp::value(nearestPoint3.X()), kEpsilon);
  EXPECT_NEAR(1.0, mp::value(nearestPoint3.Y()), kEpsilon);

  constexpr frc::Translation2d point4{-1.0 * mp::m, 2.5 * mp::m};
  auto nearestPoint4 = ellipse.Nearest(point4);
  EXPECT_NEAR(-0.8512799937611617, mp::value(nearestPoint4.X()), kEpsilon);
  EXPECT_NEAR(2.378405333174535, mp::value(nearestPoint4.Y()), kEpsilon);
}

TEST(Ellipse2dTest, Equals) {
  constexpr frc::Pose2d center1{1.0 * mp::m, 2.0 * mp::m, 90.0 * mp::deg};
  constexpr frc::Ellipse2d ellipse1{center1, 2.0 * mp::m, 3.0 * mp::m};

  constexpr frc::Pose2d center2{1.0 * mp::m, 2.0 * mp::m, 90.0 * mp::deg};
  constexpr frc::Ellipse2d ellipse2{center2, 2.0 * mp::m, 3.0 * mp::m};

  constexpr frc::Pose2d center3{1.0 * mp::m, 2.0 * mp::m, 90.0 * mp::deg};
  constexpr frc::Ellipse2d ellipse3{center3, 3.0 * mp::m, 2.0 * mp::m};

  EXPECT_EQ(ellipse1, ellipse2);
  EXPECT_NE(ellipse1, ellipse3);
  EXPECT_NE(ellipse3, ellipse2);
}
