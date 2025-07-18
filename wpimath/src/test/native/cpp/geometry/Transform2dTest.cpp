// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include <gtest/gtest.h>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Transform2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/units.h"

using namespace frc;

TEST(Transform2dTest, ToMatrix) {
  Transform2d before{1.0 * mp::m, 2.0 * mp::m, 20.0 * mp::deg};
  Transform2d after{before.ToMatrix()};

  EXPECT_EQ(before, after);
}

TEST(Transform2dTest, Inverse) {
  const Pose2d initial{1.0 * mp::m, 2.0 * mp::m, 45.0 * mp::deg};
  const Transform2d transform{{5.0 * mp::m, 0.0 * mp::m}, 5.0 * mp::deg};

  auto transformed = initial + transform;
  auto untransformed = transformed + transform.Inverse();

  EXPECT_EQ(initial, untransformed);
}

TEST(Transform2dTest, Composition) {
  const Pose2d initial{1.0 * mp::m, 2.0 * mp::m, 45.0 * mp::deg};
  const Transform2d transform1{{5.0 * mp::m, 0.0 * mp::m}, 5.0 * mp::deg};
  const Transform2d transform2{{0.0 * mp::m, 2.0 * mp::m}, 5.0 * mp::deg};

  auto transformedSeparate = initial + transform1 + transform2;
  auto transformedCombined = initial + (transform1 + transform2);

  EXPECT_EQ(transformedSeparate, transformedCombined);
}

TEST(Transform2dTest, Constexpr) {
  constexpr Transform2d defaultCtor;
  constexpr Transform2d translationRotationCtor{Translation2d{},
                                                Rotation2d{10.0 * mp::deg}};
  constexpr auto multiplied = translationRotationCtor * 5;
  constexpr auto divided = translationRotationCtor / 2;

  static_assert(defaultCtor.Translation().X() == 0.0 * mp::m);
  static_assert(translationRotationCtor.X() == 0.0 * mp::m);
  static_assert(translationRotationCtor.Y() == 0.0 * mp::m);
  static_assert(multiplied.Rotation().Degrees() == 50.0 * mp::deg);
  static_assert(translationRotationCtor.Inverse().Rotation().Degrees() ==
                (-10.0 * mp::deg));
  static_assert(translationRotationCtor.Inverse().X() == 0.0 * mp::m);
  static_assert(translationRotationCtor.Inverse().Y() == 0.0 * mp::m);
  static_assert(divided.Rotation().Degrees() == 5.0 * mp::deg);
}
