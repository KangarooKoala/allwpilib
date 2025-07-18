// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include <cstdlib>

#include <gtest/gtest.h>

#include "frc/geometry/Pose2d.h"
#include "frc/units-usc.h"
#include "frc/units.h"

using namespace frc;

TEST(Pose2dTest, RotateBy) {
  constexpr auto x = 1.0 * mp::m;
  constexpr auto y = 2.0 * mp::m;
  const Pose2d initial{x, y, 45.0 * mp::deg};

  const Rotation2d rotation{5.0 * mp::deg};
  const auto rotated = initial.RotateBy(rotation);

  // Translation is rotated by CCW rotation matrix
  double c = rotation.Cos();
  double s = rotation.Sin();
  EXPECT_DOUBLE_EQ(c * mp::value(x) - s * mp::value(y), mp::value(rotated.X()));
  EXPECT_DOUBLE_EQ(s * mp::value(x) + c * mp::value(y), mp::value(rotated.Y()));
  EXPECT_DOUBLE_EQ(
      mp::value(initial.Rotation().Degrees()) + mp::value(rotation.Degrees()),
      mp::value(rotated.Rotation().Degrees()));
}

TEST(Pose2dTest, TransformBy) {
  const Pose2d initial{1.0 * mp::m, 2.0 * mp::m, 45.0 * mp::deg};
  const Transform2d transform{Translation2d{5.0 * mp::m, 0.0 * mp::m},
                              5.0 * mp::deg};

  const auto transformed = initial + transform;

  EXPECT_DOUBLE_EQ(1.0 + 5.0 / std::sqrt(2.0), mp::value(transformed.X()));
  EXPECT_DOUBLE_EQ(2.0 + 5.0 / std::sqrt(2.0), mp::value(transformed.Y()));
  EXPECT_DOUBLE_EQ(50.0, mp::value(transformed.Rotation().Degrees()));
}

TEST(Pose2dTest, RelativeTo) {
  const Pose2d initial{0.0 * mp::m, 0.0 * mp::m, 45.0 * mp::deg};
  const Pose2d final{5.0 * mp::m, 5.0 * mp::m, 45.0 * mp::deg};

  const auto finalRelativeToInitial = final.RelativeTo(initial);

  EXPECT_NEAR(5.0 * std::sqrt(2.0), mp::value(finalRelativeToInitial.X()),
              1e-9);
  EXPECT_NEAR(0.0, mp::value(finalRelativeToInitial.Y()), 1e-9);
  EXPECT_NEAR(0.0, mp::value(finalRelativeToInitial.Rotation().Degrees()),
              1e-9);
}

TEST(Pose2dTest, RotateAround) {
  const Pose2d initial{5.0 * mp::m, 0.0 * mp::m, 0.0 * mp::deg};
  const Translation2d point{0.0 * mp::m, 0.0 * mp::m};

  const auto rotated = initial.RotateAround(point, Rotation2d{180.0 * mp::deg});

  EXPECT_NEAR(-5.0, mp::value(rotated.X()), 1e-9);
  EXPECT_NEAR(0.0, mp::value(rotated.Y()), 1e-9);
  EXPECT_NEAR(180.0, mp::value(rotated.Rotation().Degrees()), 1e-9);
}

TEST(Pose2dTest, Equality) {
  const Pose2d a{0.0 * mp::m, 5.0 * mp::m, 43.0 * mp::deg};
  const Pose2d b{0.0 * mp::m, 5.0 * mp::m, 43.0 * mp::deg};
  EXPECT_TRUE(a == b);
}

TEST(Pose2dTest, Inequality) {
  const Pose2d a{0.0 * mp::m, 5.0 * mp::m, 43.0 * mp::deg};
  const Pose2d b{0.0 * mp::m, 5.0 * mp::ft, 43.0 * mp::deg};
  EXPECT_TRUE(a != b);
}

TEST(Pose2dTest, Minus) {
  const Pose2d initial{0.0 * mp::m, 0.0 * mp::m, 45.0 * mp::deg};
  const Pose2d final{5.0 * mp::m, 5.0 * mp::m, 45.0 * mp::deg};

  const auto transform = final - initial;

  EXPECT_NEAR(5.0 * std::sqrt(2.0), mp::value(transform.X()), 1e-9);
  EXPECT_NEAR(0.0, mp::value(transform.Y()), 1e-9);
  EXPECT_NEAR(0.0, mp::value(transform.Rotation().Degrees()), 1e-9);
}

TEST(Pose2dTest, Nearest) {
  const Pose2d origin{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::deg};

  const Pose2d pose1{Translation2d{1.0 * mp::m, Rotation2d{45.0 * mp::deg}},
                     0.0 * mp::deg};
  const Pose2d pose2{Translation2d{2.0 * mp::m, Rotation2d{90.0 * mp::deg}},
                     0.0 * mp::deg};
  const Pose2d pose3{Translation2d{3.0 * mp::m, Rotation2d{135.0 * mp::deg}},
                     0.0 * mp::deg};
  const Pose2d pose4{Translation2d{4.0 * mp::m, Rotation2d{180.0 * mp::deg}},
                     0.0 * mp::deg};
  const Pose2d pose5{Translation2d{5.0 * mp::m, Rotation2d{270.0 * mp::deg}},
                     0.0 * mp::deg};

  EXPECT_DOUBLE_EQ(mp::value(pose3.X()),
                   mp::value(origin.Nearest({pose5, pose3, pose4}).X()));
  EXPECT_DOUBLE_EQ(mp::value(pose3.Y()),
                   mp::value(origin.Nearest({pose5, pose3, pose4}).Y()));

  EXPECT_DOUBLE_EQ(mp::value(pose1.X()),
                   mp::value(origin.Nearest({pose1, pose2, pose3}).X()));
  EXPECT_DOUBLE_EQ(mp::value(pose1.Y()),
                   mp::value(origin.Nearest({pose1, pose2, pose3}).Y()));

  EXPECT_DOUBLE_EQ(mp::value(pose2.X()),
                   mp::value(origin.Nearest({pose4, pose2, pose3}).X()));
  EXPECT_DOUBLE_EQ(mp::value(pose2.Y()),
                   mp::value(origin.Nearest({pose4, pose2, pose3}).Y()));

  // Rotation component sort (when distance is the same)
  // Use the same translation because using different angles at the same
  // distance can cause rounding error.
  const Translation2d translation{1.0 * mp::m, Rotation2d{0.0 * mp::deg}};

  const Pose2d poseA{translation, 0.0 * mp::deg};
  const Pose2d poseB{translation, Rotation2d{30.0 * mp::deg}};
  const Pose2d poseC{translation, Rotation2d{120.0 * mp::deg}};
  const Pose2d poseD{translation, Rotation2d{90.0 * mp::deg}};
  const Pose2d poseE{translation, Rotation2d{-180.0 * mp::deg}};

  EXPECT_DOUBLE_EQ(
      mp::value(poseA.Rotation().Degrees()),
      mp::value(Pose2d(0.0 * mp::m, 0.0 * mp::m, Rotation2d{360.0 * mp::deg})
                    .Nearest({poseA, poseB, poseD})
                    .Rotation()
                    .Degrees()));
  EXPECT_DOUBLE_EQ(
      mp::value(poseB.Rotation().Degrees()),
      mp::value(Pose2d(0.0 * mp::m, 0.0 * mp::m, Rotation2d{-335.0 * mp::deg})
                    .Nearest({poseB, poseC, poseD})
                    .Rotation()
                    .Degrees()));
  EXPECT_DOUBLE_EQ(
      mp::value(poseC.Rotation().Degrees()),
      mp::value(Pose2d(0.0 * mp::m, 0.0 * mp::m, Rotation2d{-120.0 * mp::deg})
                    .Nearest({poseB, poseC, poseD})
                    .Rotation()
                    .Degrees()));
  EXPECT_DOUBLE_EQ(
      mp::value(poseD.Rotation().Degrees()),
      mp::value(Pose2d(0.0 * mp::m, 0.0 * mp::m, Rotation2d{85.0 * mp::deg})
                    .Nearest({poseA, poseC, poseD})
                    .Rotation()
                    .Degrees()));
  EXPECT_DOUBLE_EQ(
      mp::value(poseE.Rotation().Degrees()),
      mp::value(Pose2d(0.0 * mp::m, 0.0 * mp::m, Rotation2d{170.0 * mp::deg})
                    .Nearest({poseA, poseD, poseE})
                    .Rotation()
                    .Degrees()));
}

TEST(Pose2dTest, ToMatrix) {
  Pose2d before{1.0 * mp::m, 2.0 * mp::m, 20.0 * mp::deg};
  Pose2d after{before.ToMatrix()};

  EXPECT_EQ(before, after);
}

TEST(Pose2dTest, Constexpr) {
  constexpr Pose2d defaultConstructed;
  constexpr Pose2d translationRotation{Translation2d{0.0 * mp::m, 1.0 * mp::m},
                                       Rotation2d{0.0 * mp::deg}};
  constexpr Pose2d coordRotation{0.0 * mp::m, 0.0 * mp::m,
                                 Rotation2d{45.0 * mp::deg}};

  constexpr auto added =
      translationRotation +
      Transform2d{Translation2d{}, Rotation2d{45.0 * mp::deg}};
  constexpr auto multiplied = coordRotation * 2;

  static_assert(defaultConstructed.X() == 0.0 * mp::m);
  static_assert(translationRotation.Y() == 1.0 * mp::m);
  static_assert(coordRotation.Rotation().Degrees() == 45.0 * mp::deg);
  static_assert(added.X() == 0.0 * mp::m);
  static_assert(added.Y() == 1.0 * mp::m);
  static_assert(added.Rotation().Degrees() == 45.0 * mp::deg);
  static_assert(multiplied.Rotation().Degrees() == 90.0 * mp::deg);
}
