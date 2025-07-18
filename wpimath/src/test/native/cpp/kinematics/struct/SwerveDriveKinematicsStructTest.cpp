// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "../../StructTestBase.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/struct/SwerveDriveKinematicsStruct.h"

using namespace frc;

struct SwerveDriveKinematicsStructTestData {
  using Type = SwerveDriveKinematics<4>;

  inline static const Type kTestData{
      frc::Translation2d{1.0 * mp::m, 0.9 * mp::m},
      frc::Translation2d{1.1 * mp::m, -0.8 * mp::m},
      frc::Translation2d{-1.2 * mp::m, 0.7 * mp::m},
      frc::Translation2d{-1.3 * mp::m, -0.6 * mp::m}};

  static void CheckEq(const Type& testData, const Type& data) {
    EXPECT_EQ(testData.GetModules(), data.GetModules());
  }
};

INSTANTIATE_TYPED_TEST_SUITE_P(SwerveDriveKinematics, StructTest,
                               SwerveDriveKinematicsStructTestData);
