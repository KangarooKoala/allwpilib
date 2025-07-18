// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/units.h"

TEST(SwerveModulePositionTest, Equality) {
  frc::SwerveModulePosition position1{2.0 * mp::m, 90.0 * mp::deg};
  frc::SwerveModulePosition position2{2.0 * mp::m, 90.0 * mp::deg};

  EXPECT_EQ(position1, position2);
}

TEST(SwerveModulePositionTest, Inequality) {
  frc::SwerveModulePosition position1{1.0 * mp::m, 90.0 * mp::deg};
  frc::SwerveModulePosition position2{2.0 * mp::m, 90.0 * mp::deg};
  frc::SwerveModulePosition position3{1.0 * mp::m, 89.0 * mp::deg};

  EXPECT_NE(position1, position2);
  EXPECT_NE(position1, position3);
}
