// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO
// * JNI and Java tests

#include <string>

#include <gtest/gtest.h>
#include <wpi/array.h>
#include <wpi/print.h>

#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveDesaturator.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/time.h"
#include "units/velocity.h"

static constexpr double kEpsilon = 1e-9;

frc::ChassisSpeeds Undiscretize(const frc::ChassisSpeeds& speeds,
                                units::second_t dt) {
  frc::Twist2d twist{speeds.vx * dt, speeds.vy * dt, speeds.omega * dt};
  frc::Pose2d pose = frc::Pose2d{}.Exp(twist);

  return {pose.X() / dt, pose.Y() / dt, pose.Rotation().Radians() / dt};
}

void ExpectScaled(const frc::ChassisSpeeds& lhs,
                  const frc::ChassisSpeeds& rhs) {
  std::string errorMessage = fmt::format(
      "Chassis speeds are not scalar multiples! lhs: [vx = {}, vy = {}, omega "
      "= {}], rhs: [vx = {}, vy = {}, omega = {}]",
      lhs.vx, lhs.vy, lhs.omega, rhs.vx, rhs.vy, rhs.omega);
  if (std::abs(lhs.vx.value()) < kEpsilon &&
      std::abs(lhs.vy.value()) < kEpsilon &&
      std::abs(lhs.omega.value()) < kEpsilon) {
    // lhs is zero
    return;
  }
  auto scaled = lhs * (rhs.vx / lhs.vx);
  EXPECT_NEAR(scaled.vx.value(), rhs.vx.value(), kEpsilon) << errorMessage;
  EXPECT_NEAR(scaled.vy.value(), rhs.vy.value(), kEpsilon) << errorMessage;
  EXPECT_NEAR(scaled.omega.value(), rhs.omega.value(), kEpsilon)
      << errorMessage;
}

TEST(SwerveDesaturatorTest, StraightUnsaturated) {
  frc::ChassisSpeeds speeds{0.5_mps, 0.0_mps, 0.0_rad_per_s};
  auto dt = 0.02_s;
  auto maxModuleSpeed = 1.0_mps;
  wpi::array<frc::Translation2d, 4> modules{
      frc::Translation2d{1_m, 1_m}, frc::Translation2d{1_m, -1_m},
      frc::Translation2d{-1_m, 1_m}, frc::Translation2d{-1_m, -1_m}};

  auto outputSpeeds = frc::SwerveDesaturator::DesaturatedDiscretize(speeds, dt, maxModuleSpeed, modules, true);
  auto expected = frc::ChassisSpeeds::Discretize(speeds, dt);

  EXPECT_NEAR(outputSpeeds.vx.value(), expected.vx.value(), kEpsilon);
  EXPECT_NEAR(outputSpeeds.vy.value(), expected.vy.value(), kEpsilon);
  EXPECT_NEAR(outputSpeeds.omega.value(), expected.omega.value(), kEpsilon);
}

TEST(SwerveDesaturatorTest, StraightAllSaturated) {
  frc::ChassisSpeeds speeds{2.0_mps, 1.0_mps, 0.0_rad_per_s};
  auto dt = 0.02_s;
  auto maxModuleSpeed = 1.0_mps;
  wpi::array<frc::Translation2d, 4> modules{
      frc::Translation2d{1_m, 1_m}, frc::Translation2d{1_m, -1_m},
      frc::Translation2d{-1_m, 1_m}, frc::Translation2d{-1_m, -1_m}};
  frc::SwerveDriveKinematics<4> kinematics{modules};

  auto outputSpeeds = frc::SwerveDesaturator::DesaturatedDiscretize(speeds, dt, maxModuleSpeed, modules, true);
  auto outputStates = kinematics.ToSwerveModuleStates(outputSpeeds);

  auto maxRealSpeed = 0_mps;
  for (auto module : outputStates) {
    if (module.speed > maxRealSpeed) {
      maxRealSpeed = module.speed;
    }
  }

  EXPECT_NEAR(maxModuleSpeed.value(), maxRealSpeed.value(), kEpsilon);
  ExpectScaled(speeds, Undiscretize(outputSpeeds, dt));
}

TEST(SwerveDesaturatorTest, CurvedUnsaturated) {
  frc::ChassisSpeeds speeds{0.5_mps, 0.0_mps, 0.1_rad_per_s};
  auto dt = 0.02_s;
  auto maxModuleSpeed = 1.0_mps;
  wpi::array<frc::Translation2d, 4> modules{
      frc::Translation2d{1_m, 1_m}, frc::Translation2d{1_m, -1_m},
      frc::Translation2d{-1_m, 1_m}, frc::Translation2d{-1_m, -1_m}};

  auto outputSpeeds = frc::SwerveDesaturator::DesaturatedDiscretize(speeds, dt, maxModuleSpeed, modules, true);
  auto expected = frc::ChassisSpeeds::Discretize(speeds, dt);

  EXPECT_NEAR(outputSpeeds.vx.value(), expected.vx.value(), kEpsilon);
  EXPECT_NEAR(outputSpeeds.vy.value(), expected.vy.value(), kEpsilon);
  EXPECT_NEAR(outputSpeeds.omega.value(), expected.omega.value(), kEpsilon);
}

TEST(SwerveDesaturatorTest, CurvedOneSaturated) {
  frc::ChassisSpeeds speeds{0.5_mps, -0.5_mps, 0.5_rad_per_s};
  auto dt = 0.02_s;
  auto maxModuleSpeed = 1.0_mps;
  wpi::array<frc::Translation2d, 4> modules{
      frc::Translation2d{1_m, 1_m}, frc::Translation2d{1_m, -1_m},
      frc::Translation2d{-1_m, 1_m}, frc::Translation2d{-1_m, -1_m}};
  frc::SwerveDriveKinematics<4> kinematics{modules};

  auto outputSpeeds = frc::SwerveDesaturator::DesaturatedDiscretize(speeds, dt, maxModuleSpeed, modules, true);
  auto outputStates = kinematics.ToSwerveModuleStates(outputSpeeds);

  auto maxRealSpeed = 0_mps;
  for (auto module : outputStates) {
    if (module.speed > maxRealSpeed) {
      maxRealSpeed = module.speed;
    }
  }

  EXPECT_NEAR(maxModuleSpeed.value(), maxRealSpeed.value(), kEpsilon);
  ExpectScaled(speeds, Undiscretize(outputSpeeds, dt));
}

TEST(SwerveDesaturatorTest, CurvedAllSaturated) {
  frc::ChassisSpeeds speeds{2.0_mps, 1.0_mps, 0.1_rad_per_s};
  auto dt = 0.02_s;
  auto maxModuleSpeed = 1.0_mps;
  wpi::array<frc::Translation2d, 4> modules{
      frc::Translation2d{1_m, 1_m}, frc::Translation2d{1_m, -1_m},
      frc::Translation2d{-1_m, 1_m}, frc::Translation2d{-1_m, -1_m}};
  frc::SwerveDriveKinematics<4> kinematics{modules};

  auto outputSpeeds = frc::SwerveDesaturator::DesaturatedDiscretize(speeds, dt, maxModuleSpeed, modules, true);
  auto outputStates = kinematics.ToSwerveModuleStates(outputSpeeds);

  auto maxRealSpeed = 0_mps;
  for (auto module : outputStates) {
    if (module.speed > maxRealSpeed) {
      maxRealSpeed = module.speed;
    }
  }

  EXPECT_NEAR(maxModuleSpeed.value(), maxRealSpeed.value(), kEpsilon);
  ExpectScaled(speeds, Undiscretize(outputSpeeds, dt));
}

TEST(SwerveDesaturatorTest, ReverseSlope) {
  frc::ChassisSpeeds speeds{6_mps, -20_mps, 20_rad_per_s};
  auto dt = 0.02_s;
  auto maxModuleSpeed = 2.0_mps;
  wpi::array<frc::Translation2d, 4> modules{
      frc::Translation2d{1_m, 0_m}, frc::Translation2d{1_m, 0_m},
      frc::Translation2d{1_m, 0_m}, frc::Translation2d{1_m, 0_m}};
  frc::SwerveDriveKinematics<4> kinematics{modules};

  auto outputSpeeds = frc::SwerveDesaturator::DesaturatedDiscretize(speeds, dt, maxModuleSpeed, modules, true);
  auto outputStates = kinematics.ToSwerveModuleStates(outputSpeeds);

  auto maxRealSpeed = 0_mps;
  for (auto module : outputStates) {
    if (module.speed > maxRealSpeed) {
      maxRealSpeed = module.speed;
    }
  }

  EXPECT_NEAR(maxModuleSpeed.value(), maxRealSpeed.value(), kEpsilon);
  ExpectScaled(speeds, Undiscretize(outputSpeeds, dt));
}

TEST(SwerveDesaturatorTest, PositiveLocalMin) {
  frc::ChassisSpeeds speeds{6_mps, -20_mps, 20_rad_per_s};
  auto dt = 0.02_s;
  auto maxModuleSpeed = 1.0_mps;
  wpi::array<frc::Translation2d, 4> modules{
      frc::Translation2d{1_m, 0_m}, frc::Translation2d{1_m, 0_m},
      frc::Translation2d{1_m, 0_m}, frc::Translation2d{1_m, 0_m}};
  frc::SwerveDriveKinematics<4> kinematics{modules};

  auto outputSpeeds = frc::SwerveDesaturator::DesaturatedDiscretize(speeds, dt, maxModuleSpeed, modules, true);
  auto outputStates = kinematics.ToSwerveModuleStates(outputSpeeds);

  auto maxRealSpeed = 0_mps;
  for (auto module : outputStates) {
    if (module.speed > maxRealSpeed) {
      maxRealSpeed = module.speed;
    }
  }

  EXPECT_NEAR(maxModuleSpeed.value(), maxRealSpeed.value(), kEpsilon);
  ExpectScaled(speeds, Undiscretize(outputSpeeds, dt));
}

TEST(SwerveDesaturatorTest, SeparateRanges) {
  // TODO More reasonable speeds
  // TODO Document how I found these inputs
  frc::ChassisSpeeds speeds{9_mps, -30_mps, 30_rad_per_s};
  auto dt = 0.02_s;
  auto maxModuleSpeed = 2.0_mps;
  wpi::array<frc::Translation2d, 4> modules{
      frc::Translation2d{1_m, 0_m}, frc::Translation2d{1_m, 0_m},
      frc::Translation2d{1_m, 0_m}, frc::Translation2d{1_m, 0_m}};
  frc::SwerveDriveKinematics<4> kinematics{modules};

  auto outputSpeeds = frc::SwerveDesaturator::DesaturatedDiscretize(speeds, dt, maxModuleSpeed, modules, true);
  auto expected = frc::ChassisSpeeds::Discretize(speeds, dt);

  EXPECT_NEAR(outputSpeeds.vx.value(), expected.vx.value(), kEpsilon);
  EXPECT_NEAR(outputSpeeds.vy.value(), expected.vy.value(), kEpsilon);
  EXPECT_NEAR(outputSpeeds.omega.value(), expected.omega.value(), kEpsilon);
}

TEST(SwerveDesaturatorTest, InvalidatedResult) {
  frc::ChassisSpeeds speeds{5_mps, -14_mps, 30_rad_per_s};
  auto dt = 0.02_s;
  auto maxModuleSpeed = 1.3_mps;
  wpi::array<frc::Translation2d, 4> modules{
      frc::Translation2d{0.5_m, 0_m}, frc::Translation2d{0.5_m, 0_m},
      frc::Translation2d{0.5_m, 0.2_m}, frc::Translation2d{0.5_m, 0.2_m}};
  frc::SwerveDriveKinematics<4> kinematics{modules};

  auto outputSpeeds = frc::SwerveDesaturator::DesaturatedDiscretize(speeds, dt, maxModuleSpeed, modules, true);
  auto outputStates = kinematics.ToSwerveModuleStates(outputSpeeds);

  auto maxRealSpeed = 0_mps;
  for (auto module : outputStates) {
    if (module.speed > maxRealSpeed) {
      maxRealSpeed = module.speed;
    }
  }

  EXPECT_NEAR(maxModuleSpeed.value(), maxRealSpeed.value(), kEpsilon);
  ExpectScaled(speeds, Undiscretize(outputSpeeds, dt));
}
