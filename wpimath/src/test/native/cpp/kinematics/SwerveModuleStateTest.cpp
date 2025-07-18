// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/units.h"

static constexpr double kEpsilon = 1E-9;

TEST(SwerveModuleStateTest, Optimize) {
  frc::Rotation2d angleA{45.0 * mp::deg};
  frc::SwerveModuleState refA{-2.0 * mp::m / mp::s, 180.0 * mp::deg};
  refA.Optimize(angleA);

  EXPECT_NEAR(mp::value(refA.speed), 2.0, kEpsilon);
  EXPECT_NEAR(mp::value(refA.angle.Degrees()), 0.0, kEpsilon);

  frc::Rotation2d angleB{-50.0 * mp::deg};
  frc::SwerveModuleState refB{4.7 * mp::m / mp::s, 41.0 * mp::deg};
  refB.Optimize(angleB);

  EXPECT_NEAR(mp::value(refB.speed), -4.7, kEpsilon);
  EXPECT_NEAR(mp::value(refB.angle.Degrees()), -139.0, kEpsilon);
}

TEST(SwerveModuleStateTest, NoOptimize) {
  frc::Rotation2d angleA{0.0 * mp::deg};
  frc::SwerveModuleState refA{2.0 * mp::m / mp::s, 89.0 * mp::deg};
  refA.Optimize(angleA);

  EXPECT_NEAR(mp::value(refA.speed), 2.0, kEpsilon);
  EXPECT_NEAR(mp::value(refA.angle.Degrees()), 89.0, kEpsilon);

  frc::Rotation2d angleB{0.0 * mp::deg};
  frc::SwerveModuleState refB{-2.0 * mp::m / mp::s, -2.0 * mp::deg};
  refB.Optimize(angleB);

  EXPECT_NEAR(mp::value(refB.speed), -2.0, kEpsilon);
  EXPECT_NEAR(mp::value(refB.angle.Degrees()), -2.0, kEpsilon);
}

TEST(SwerveModuleStateTest, CosineScaling) {
  frc::Rotation2d angleA{0.0 * mp::deg};
  frc::SwerveModuleState refA{2.0 * mp::m / mp::s, 45.0 * mp::deg};
  refA.CosineScale(angleA);

  EXPECT_NEAR(mp::value(refA.speed), std::sqrt(2.0), kEpsilon);
  EXPECT_NEAR(mp::value(refA.angle.Degrees()), 45.0, kEpsilon);

  frc::Rotation2d angleB{45.0 * mp::deg};
  frc::SwerveModuleState refB{2.0 * mp::m / mp::s, 45.0 * mp::deg};
  refB.CosineScale(angleB);

  EXPECT_NEAR(mp::value(refB.speed), 2.0, kEpsilon);
  EXPECT_NEAR(mp::value(refB.angle.Degrees()), 45.0, kEpsilon);

  frc::Rotation2d angleC{-45.0 * mp::deg};
  frc::SwerveModuleState refC{2.0 * mp::m / mp::s, 45.0 * mp::deg};
  refC.CosineScale(angleC);

  EXPECT_NEAR(mp::value(refC.speed), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(refC.angle.Degrees()), 45.0, kEpsilon);

  frc::Rotation2d angleD{135.0 * mp::deg};
  frc::SwerveModuleState refD{2.0 * mp::m / mp::s, 45.0 * mp::deg};
  refD.CosineScale(angleD);

  EXPECT_NEAR(mp::value(refD.speed), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(refD.angle.Degrees()), 45.0, kEpsilon);

  frc::Rotation2d angleE{-135.0 * mp::deg};
  frc::SwerveModuleState refE{2.0 * mp::m / mp::s, 45.0 * mp::deg};
  refE.CosineScale(angleE);

  EXPECT_NEAR(mp::value(refE.speed), -2.0, kEpsilon);
  EXPECT_NEAR(mp::value(refE.angle.Degrees()), 45.0, kEpsilon);

  frc::Rotation2d angleF{180.0 * mp::deg};
  frc::SwerveModuleState refF{2.0 * mp::m / mp::s, 45.0 * mp::deg};
  refF.CosineScale(angleF);

  EXPECT_NEAR(mp::value(refF.speed), -std::sqrt(2.0), kEpsilon);
  EXPECT_NEAR(mp::value(refF.angle.Degrees()), 45.0, kEpsilon);

  frc::Rotation2d angleG{0.0 * mp::deg};
  frc::SwerveModuleState refG{-2.0 * mp::m / mp::s, 45.0 * mp::deg};
  refG.CosineScale(angleG);

  EXPECT_NEAR(mp::value(refG.speed), -std::sqrt(2.0), kEpsilon);
  EXPECT_NEAR(mp::value(refG.angle.Degrees()), 45.0, kEpsilon);

  frc::Rotation2d angleH{45.0 * mp::deg};
  frc::SwerveModuleState refH{-2.0 * mp::m / mp::s, 45.0 * mp::deg};
  refH.CosineScale(angleH);

  EXPECT_NEAR(mp::value(refH.speed), -2.0, kEpsilon);
  EXPECT_NEAR(mp::value(refH.angle.Degrees()), 45.0, kEpsilon);

  frc::Rotation2d angleI{-45.0 * mp::deg};
  frc::SwerveModuleState refI{-2.0 * mp::m / mp::s, 45.0 * mp::deg};
  refI.CosineScale(angleI);

  EXPECT_NEAR(mp::value(refI.speed), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(refI.angle.Degrees()), 45.0, kEpsilon);

  frc::Rotation2d angleJ{135.0 * mp::deg};
  frc::SwerveModuleState refJ{-2.0 * mp::m / mp::s, 45.0 * mp::deg};
  refJ.CosineScale(angleJ);

  EXPECT_NEAR(mp::value(refJ.speed), 0.0, kEpsilon);
  EXPECT_NEAR(mp::value(refJ.angle.Degrees()), 45.0, kEpsilon);

  frc::Rotation2d angleK{-135.0 * mp::deg};
  frc::SwerveModuleState refK{-2.0 * mp::m / mp::s, 45.0 * mp::deg};
  refK.CosineScale(angleK);

  EXPECT_NEAR(mp::value(refK.speed), 2.0, kEpsilon);
  EXPECT_NEAR(mp::value(refK.angle.Degrees()), 45.0, kEpsilon);

  frc::Rotation2d angleL{180.0 * mp::deg};
  frc::SwerveModuleState refL{-2.0 * mp::m / mp::s, 45.0 * mp::deg};
  refL.CosineScale(angleL);

  EXPECT_NEAR(mp::value(refL.speed), std::sqrt(2.0), kEpsilon);
  EXPECT_NEAR(mp::value(refL.angle.Degrees()), 45.0, kEpsilon);
}

TEST(SwerveModuleStateTest, Equality) {
  frc::SwerveModuleState state1{2.0 * mp::m / mp::s, 90.0 * mp::deg};
  frc::SwerveModuleState state2{2.0 * mp::m / mp::s, 90.0 * mp::deg};

  EXPECT_EQ(state1, state2);
}

TEST(SwerveModuleStateTest, Inequality) {
  frc::SwerveModuleState state1{1.0 * mp::m / mp::s, 90.0 * mp::deg};
  frc::SwerveModuleState state2{2.0 * mp::m / mp::s, 90.0 * mp::deg};
  frc::SwerveModuleState state3{1.0 * mp::m / mp::s, 89.0 * mp::deg};

  EXPECT_NE(state1, state2);
  EXPECT_NE(state1, state3);
}
