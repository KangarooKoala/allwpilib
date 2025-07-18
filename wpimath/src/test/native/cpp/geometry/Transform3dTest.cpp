// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include <gtest/gtest.h>

#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Transform3d.h"
#include "frc/geometry/Translation3d.h"
#include "frc/units.h"

using namespace frc;

TEST(Transform3dTest, ToMatrix) {
  Transform3d before{
      1.0 * mp::m, 2.0 * mp::m, 3.0 * mp::m,
      Rotation3d{10.0 * mp::deg, 20.0 * mp::deg, 30.0 * mp::deg}};
  Transform3d after{before.ToMatrix()};

  EXPECT_EQ(before, after);
}

TEST(Transform3dTest, Inverse) {
  Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  const Pose3d initial{1.0 * mp::m, 2.0 * mp::m, 3.0 * mp::m,
                       Rotation3d{zAxis, 45.0 * mp::deg}};
  const Transform3d transform{{5.0 * mp::m, 4.0 * mp::m, 3.0 * mp::m},
                              Rotation3d{zAxis, 5.0 * mp::deg}};

  auto transformed = initial + transform;
  auto untransformed = transformed + transform.Inverse();

  EXPECT_EQ(initial, untransformed);
}

TEST(Transform3dTest, Composition) {
  Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  const Pose3d initial{1.0 * mp::m, 2.0 * mp::m, 3.0 * mp::m,
                       Rotation3d{zAxis, 45.0 * mp::deg}};
  const Transform3d transform1{{5.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m},
                               Rotation3d{zAxis, 5.0 * mp::deg}};
  const Transform3d transform2{{0.0 * mp::m, 2.0 * mp::m, 0.0 * mp::m},
                               Rotation3d{zAxis, 5.0 * mp::deg}};

  auto transformedSeparate = initial + transform1 + transform2;
  auto transformedCombined = initial + (transform1 + transform2);

  EXPECT_EQ(transformedSeparate, transformedCombined);
}
