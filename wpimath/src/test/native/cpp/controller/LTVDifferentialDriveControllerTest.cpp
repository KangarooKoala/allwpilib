// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include <gtest/gtest.h>

#include "frc/MathUtil.h"
#include "frc/controller/LTVDifferentialDriveController.h"
#include "frc/system/NumericalIntegration.h"
#include "frc/system/plant/LinearSystemId.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/units.h"

#define EXPECT_NEAR_UNITS(val1, val2, eps) EXPECT_LE(mp::abs(val1 - val2), eps)

static constexpr mp::quantity<mp::m> kTolerance = (1.0 / 12.0) * mp::m;
static constexpr mp::quantity<mp::rad> kAngularTolerance =
    2.0 * std::numbers::pi / 180.0 * mp::rad;

/**
 * States of the drivetrain system.
 */
class State {
 public:
  /// X position in global coordinate frame.
  static constexpr int kX = 0;

  /// Y position in global coordinate frame.
  static constexpr int kY = 1;

  /// Heading in global coordinate frame.
  static constexpr int kHeading = 2;

  /// Left encoder velocity.
  static constexpr int kLeftVelocity = 3;

  /// Right encoder velocity.
  static constexpr int kRightVelocity = 4;
};

static constexpr auto kLinearV = 3.02 * mp::V / (mp::m / mp::s);
static constexpr auto kLinearA = 0.642 * mp::V / (mp::m / mp::s2);
static constexpr auto kAngularV = 1.382 * mp::V / (mp::m / mp::s);
static constexpr auto kAngularA = 0.08495 * mp::V / (mp::m / mp::s2);
static auto plant = frc::LinearSystemId::IdentifyDrivetrainSystem(
    kLinearV, kLinearA, kAngularV, kAngularA);
static constexpr auto kTrackwidth = 0.9 * mp::m;

frc::Vectord<5> Dynamics(const frc::Vectord<5>& x, const frc::Vectord<2>& u) {
  double v = (x(State::kLeftVelocity) + x(State::kRightVelocity)) / 2.0;

  frc::Vectord<5> xdot;
  xdot(0) = v * std::cos(x(State::kHeading));
  xdot(1) = v * std::sin(x(State::kHeading));
  xdot(2) = mp::value((x(State::kRightVelocity) - x(State::kLeftVelocity)) /
                      kTrackwidth);
  xdot.block<2, 1>(3, 0) = plant.A() * x.block<2, 1>(3, 0) + plant.B() * u;
  return xdot;
}

TEST(LTVDifferentialDriveControllerTest, ReachesReference) {
  constexpr mp::quantity<mp::s> kDt = 20.0 * mp::ms;

  frc::LTVDifferentialDriveController controller{
      plant, kTrackwidth, {0.0625, 0.125, 2.5, 0.95, 0.95}, {12.0, 12.0}, kDt};
  frc::Pose2d robotPose{2.7 * mp::m, 23.0 * mp::m, 0.0 * mp::deg};

  auto waypoints =
      std::vector{frc::Pose2d{2.75 * mp::m, 22.521 * mp::m, 0.0 * mp::rad},
                  frc::Pose2d{24.73 * mp::m, 19.68 * mp::m, 5.846 * mp::rad}};
  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      waypoints, {8.8 * mp::m / mp::s, 0.1 * mp::m / mp::s2});

  frc::Vectord<5> x = frc::Vectord<5>::Zero();
  x(State::kX) = mp::value(robotPose.X());
  x(State::kY) = mp::value(robotPose.Y());
  x(State::kHeading) = mp::value(robotPose.Rotation().Radians());

  auto totalTime = trajectory.TotalTime();
  for (size_t i = 0; i < mp::value(totalTime / kDt); ++i) {
    auto state = trajectory.Sample(kDt * i);
    robotPose = frc::Pose2d{x(State::kX) * mp::m, x(State::kY) * mp::m,
                            x(State::kHeading) * mp::rad};
    auto [leftVoltage, rightVoltage] =
        controller.Calculate(robotPose, x(State::kLeftVelocity) * mp::m / mp::s,
                             x(State::kRightVelocity) * mp::m / mp::s, state);

    x = frc::RKDP(
        &Dynamics, x,
        frc::Vectord<2>{mp::value(leftVoltage), mp::value(rightVoltage)}, kDt);
  }

  auto& endPose = trajectory.States().back().pose;
  EXPECT_NEAR_UNITS(endPose.X(), robotPose.X(), kTolerance);
  EXPECT_NEAR_UNITS(endPose.Y(), robotPose.Y(), kTolerance);
  EXPECT_NEAR_UNITS(frc::AngleModulus(endPose.Rotation().Radians() -
                                      robotPose.Rotation().Radians()),
                    0.0 * mp::rad, kAngularTolerance);
}
