// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/kinematics/DifferentialDriveWheelSpeeds.h"
#include "frc/units.h"

TEST(DifferentialDriveWheelSpeedsTest, Plus) {
  const frc::DifferentialDriveWheelSpeeds left{1.0 * mp::m / mp::s,
                                               0.5 * mp::m / mp::s};
  const frc::DifferentialDriveWheelSpeeds right{2.0 * mp::m / mp::s,
                                                1.5 * mp::m / mp::s};

  const frc::DifferentialDriveWheelSpeeds result = left + right;

  EXPECT_EQ(3.0, mp::value(result.left));
  EXPECT_EQ(2.0, mp::value(result.right));
}

TEST(DifferentialDriveWheelSpeedsTest, Minus) {
  const frc::DifferentialDriveWheelSpeeds left{1.0 * mp::m / mp::s,
                                               0.5 * mp::m / mp::s};
  const frc::DifferentialDriveWheelSpeeds right{2.0 * mp::m / mp::s,
                                                0.5 * mp::m / mp::s};

  const frc::DifferentialDriveWheelSpeeds result = left - right;

  EXPECT_EQ(-1.0, mp::value(result.left));
  EXPECT_EQ(0, mp::value(result.right));
}

TEST(DifferentialDriveWheelSpeedsTest, UnaryMinus) {
  const frc::DifferentialDriveWheelSpeeds speeds{1.0 * mp::m / mp::s,
                                                 0.5 * mp::m / mp::s};

  const frc::DifferentialDriveWheelSpeeds result = -speeds;

  EXPECT_EQ(-1.0, mp::value(result.left));
  EXPECT_EQ(-0.5, mp::value(result.right));
}

TEST(DifferentialDriveWheelSpeedsTest, Multiplication) {
  const frc::DifferentialDriveWheelSpeeds speeds{1.0 * mp::m / mp::s,
                                                 0.5 * mp::m / mp::s};

  const frc::DifferentialDriveWheelSpeeds result = speeds * 2;

  EXPECT_EQ(2.0, mp::value(result.left));
  EXPECT_EQ(1.0, mp::value(result.right));
}

TEST(DifferentialDriveWheelSpeedsTest, Division) {
  const frc::DifferentialDriveWheelSpeeds speeds{1.0 * mp::m / mp::s,
                                                 0.5 * mp::m / mp::s};

  const frc::DifferentialDriveWheelSpeeds result = speeds / 2;

  EXPECT_EQ(0.5, mp::value(result.left));
  EXPECT_EQ(0.25, mp::value(result.right));
}
