// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/geometry/Rectangle2d.h"
#include "frc/units.h"

TEST(Rectangle2dTest, NewWithCorners) {
  constexpr frc::Translation2d cornerA{1.0 * mp::m, 2.0 * mp::m};
  constexpr frc::Translation2d cornerB{4.0 * mp::m, 6.0 * mp::m};

  frc::Rectangle2d rect{cornerA, cornerB};

  EXPECT_EQ(3.0, mp::value(rect.XWidth()));
  EXPECT_EQ(4.0, mp::value(rect.YWidth()));
  EXPECT_EQ(2.5, mp::value(rect.Center().X()));
  EXPECT_EQ(4.0, mp::value(rect.Center().Y()));
}

TEST(Rectangle2dTest, Intersects) {
  constexpr frc::Pose2d center{4.0 * mp::m, 3.0 * mp::m, 90.0 * mp::deg};
  constexpr frc::Rectangle2d rect{center, 2.0 * mp::m, 3.0 * mp::m};

  EXPECT_TRUE(rect.Intersects(frc::Translation2d{5.5 * mp::m, 4.0 * mp::m}));
  EXPECT_TRUE(rect.Intersects(frc::Translation2d{3.0 * mp::m, 2.0 * mp::m}));
  EXPECT_FALSE(rect.Intersects(frc::Translation2d{4.0 * mp::m, 1.5 * mp::m}));
  EXPECT_FALSE(rect.Intersects(frc::Translation2d{4.0 * mp::m, 3.5 * mp::m}));
}

TEST(Rectangle2dTest, Contains) {
  constexpr frc::Pose2d center{2.0 * mp::m, 3.0 * mp::m, 45.0 * mp::deg};
  constexpr frc::Rectangle2d rect{center, 3.0 * mp::m, 1.0 * mp::m};

  EXPECT_TRUE(rect.Contains(frc::Translation2d{2.0 * mp::m, 3.0 * mp::m}));
  EXPECT_TRUE(rect.Contains(frc::Translation2d{3.0 * mp::m, 4.0 * mp::m}));
  EXPECT_FALSE(rect.Contains(frc::Translation2d{3.0 * mp::m, 3.0 * mp::m}));
}

TEST(Rectangle2dTest, Distance) {
  constexpr double kEpsilon = 1E-9;

  constexpr frc::Pose2d center{1.0 * mp::m, 2.0 * mp::m, 270.0 * mp::deg};
  constexpr frc::Rectangle2d rect{center, 1.0 * mp::m, 2.0 * mp::m};

  constexpr frc::Translation2d point1{2.5 * mp::m, 2.0 * mp::m};
  EXPECT_NEAR(0.5, mp::value(rect.Distance(point1)), kEpsilon);

  constexpr frc::Translation2d point2{1.0 * mp::m, 2.0 * mp::m};
  EXPECT_NEAR(0, mp::value(rect.Distance(point2)), kEpsilon);

  constexpr frc::Translation2d point3{1.0 * mp::m, 1.0 * mp::m};
  EXPECT_NEAR(0.5, mp::value(rect.Distance(point3)), kEpsilon);

  constexpr frc::Translation2d point4{-1.0 * mp::m, 2.5 * mp::m};
  EXPECT_NEAR(1, mp::value(rect.Distance(point4)), kEpsilon);
}

TEST(Rectangle2dTest, Nearest) {
  constexpr double kEpsilon = 1E-9;

  constexpr frc::Pose2d center{1.0 * mp::m, 1.0 * mp::m, 90.0 * mp::deg};
  constexpr frc::Rectangle2d rect{center, 3.0 * mp::m, 4.0 * mp::m};

  constexpr frc::Translation2d point1{1.0 * mp::m, 3.0 * mp::m};
  auto nearestPoint1 = rect.Nearest(point1);
  EXPECT_NEAR(1.0, mp::value(nearestPoint1.X()), kEpsilon);
  EXPECT_NEAR(2.5, mp::value(nearestPoint1.Y()), kEpsilon);

  constexpr frc::Translation2d point2{0.0 * mp::m, 0.0 * mp::m};
  auto nearestPoint2 = rect.Nearest(point2);
  EXPECT_NEAR(0.0, mp::value(nearestPoint2.X()), kEpsilon);
  EXPECT_NEAR(0.0, mp::value(nearestPoint2.Y()), kEpsilon);
}

TEST(Rectangle2dTest, Equals) {
  constexpr frc::Pose2d center1{2.0 * mp::m, 3.0 * mp::m, 0.0 * mp::deg};
  constexpr frc::Rectangle2d rect1{center1, 5.0 * mp::m, 3.0 * mp::m};

  constexpr frc::Pose2d center2{2.0 * mp::m, 3.0 * mp::m, 0.0 * mp::deg};
  constexpr frc::Rectangle2d rect2{center2, 5.0 * mp::m, 3.0 * mp::m};

  constexpr frc::Pose2d center3{2.0 * mp::m, 3.0 * mp::m, 0.0 * mp::deg};
  constexpr frc::Rectangle2d rect3{center3, 3.0 * mp::m, 3.0 * mp::m};

  EXPECT_EQ(rect1, rect2);
  EXPECT_NE(rect2, rect3);
}
