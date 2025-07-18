// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include <gtest/gtest.h>

#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/units.h"

static constexpr double kEpsilon = 1E-9;

TEST(ChassisSpeedsTest, Discretize) {
  constexpr frc::ChassisSpeeds target{1.0 * mp::m / mp::s, 0.0 * mp::m / mp::s,
                                      0.5 * mp::rad / mp::s};
  constexpr mp::quantity<mp::s> duration = 1.0 * mp::s;
  constexpr mp::quantity<mp::s> dt = 10.0 * mp::ms;

  const auto speeds = target.Discretize(duration);
  const frc::Twist2d twist{speeds.vx * dt, speeds.vy * dt, speeds.omega * dt};

  frc::Pose2d pose;
  for (mp::quantity<mp::s> time = 0.0 * mp::s; time < duration; time += dt) {
    pose = pose + twist.Exp();
  }

  EXPECT_NEAR(mp::value(target.vx * duration), mp::value(pose.X()), kEpsilon);
  EXPECT_NEAR(mp::value(target.vy * duration), mp::value(pose.Y()), kEpsilon);
  EXPECT_NEAR(mp::value(target.omega * duration),
              mp::value(pose.Rotation().Radians()), kEpsilon);
}

TEST(ChassisSpeedsTest, ToRobotRelative) {
  const auto chassisSpeeds =
      frc::ChassisSpeeds{1.0 * mp::m / mp::s, 0.0 * mp::m / mp::s,
                         0.5 * mp::rad / mp::s}
          .ToRobotRelative(-90.0 * mp::deg);

  EXPECT_NEAR(0.0, mp::value(chassisSpeeds.vx), kEpsilon);
  EXPECT_NEAR(1.0, mp::value(chassisSpeeds.vy), kEpsilon);
  EXPECT_NEAR(0.5, mp::value(chassisSpeeds.omega), kEpsilon);
}

TEST(ChassisSpeedsTest, ToFieldRelative) {
  const auto chassisSpeeds =
      frc::ChassisSpeeds{1.0 * mp::m / mp::s, 0.0 * mp::m / mp::s,
                         0.5 * mp::rad / mp::s}
          .ToFieldRelative(45.0 * mp::deg);

  EXPECT_NEAR(1.0 / std::sqrt(2.0), mp::value(chassisSpeeds.vx), kEpsilon);
  EXPECT_NEAR(1.0 / std::sqrt(2.0), mp::value(chassisSpeeds.vy), kEpsilon);
  EXPECT_NEAR(0.5, mp::value(chassisSpeeds.omega), kEpsilon);
}

TEST(ChassisSpeedsTest, Plus) {
  const frc::ChassisSpeeds left{1.0 * mp::m / mp::s, 0.5 * mp::m / mp::s,
                                0.75 * mp::rad / mp::s};
  const frc::ChassisSpeeds right{2.0 * mp::m / mp::s, 1.5 * mp::m / mp::s,
                                 0.25 * mp::rad / mp::s};

  const frc::ChassisSpeeds result = left + right;

  EXPECT_NEAR(3.0, mp::value(result.vx), kEpsilon);
  EXPECT_NEAR(2.0, mp::value(result.vy), kEpsilon);
  EXPECT_NEAR(1.0, mp::value(result.omega), kEpsilon);
}

TEST(ChassisSpeedsTest, Minus) {
  const frc::ChassisSpeeds left{1.0 * mp::m / mp::s, 0.5 * mp::m / mp::s,
                                0.75 * mp::rad / mp::s};
  const frc::ChassisSpeeds right{2.0 * mp::m / mp::s, 0.5 * mp::m / mp::s,
                                 0.25 * mp::rad / mp::s};

  const frc::ChassisSpeeds result = left - right;

  EXPECT_NEAR(-1.0, mp::value(result.vx), kEpsilon);
  EXPECT_NEAR(0, mp::value(result.vy), kEpsilon);
  EXPECT_NEAR(0.5, mp::value(result.omega), kEpsilon);
}

TEST(ChassisSpeedsTest, UnaryMinus) {
  const frc::ChassisSpeeds speeds{1.0 * mp::m / mp::s, 0.5 * mp::m / mp::s,
                                  0.75 * mp::rad / mp::s};

  const frc::ChassisSpeeds result = -speeds;

  EXPECT_NEAR(-1.0, mp::value(result.vx), kEpsilon);
  EXPECT_NEAR(-0.5, mp::value(result.vy), kEpsilon);
  EXPECT_NEAR(-0.75, mp::value(result.omega), kEpsilon);
}

TEST(ChassisSpeedsTest, Multiplication) {
  const frc::ChassisSpeeds speeds{1.0 * mp::m / mp::s, 0.5 * mp::m / mp::s,
                                  0.75 * mp::rad / mp::s};

  const frc::ChassisSpeeds result = speeds * 2;

  EXPECT_NEAR(2.0, mp::value(result.vx), kEpsilon);
  EXPECT_NEAR(1.0, mp::value(result.vy), kEpsilon);
  EXPECT_NEAR(1.5, mp::value(result.omega), kEpsilon);
}

TEST(ChassisSpeedsTest, Division) {
  const frc::ChassisSpeeds speeds{1.0 * mp::m / mp::s, 0.5 * mp::m / mp::s,
                                  0.75 * mp::rad / mp::s};

  const frc::ChassisSpeeds result = speeds / 2;

  EXPECT_NEAR(0.5, mp::value(result.vx), kEpsilon);
  EXPECT_NEAR(0.25, mp::value(result.vy), kEpsilon);
  EXPECT_NEAR(0.375, mp::value(result.omega), kEpsilon);
}
