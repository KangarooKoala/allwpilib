// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>

#include <wpi/SymbolExports.h>

#include "frc/kinematics/MecanumDriveKinematics.h"
#include "frc/trajectory/constraint/TrajectoryConstraint.h"
#include "frc/units.h"

namespace frc {
/**
 * A class that enforces constraints on the mecanum drive kinematics.
 * This can be used to ensure that the trajectory is constructed so that the
 * commanded velocities for wheels of the drivetrain stay below a certain
 * limit.
 */
class WPILIB_DLLEXPORT MecanumDriveKinematicsConstraint
    : public TrajectoryConstraint {
 public:
  MecanumDriveKinematicsConstraint(const MecanumDriveKinematics& kinematics,
                                   mp::quantity<mp::m / mp::s> maxSpeed)
      : m_kinematics(kinematics), m_maxSpeed(maxSpeed) {}

  mp::quantity<mp::m / mp::s> MaxVelocity(
      const Pose2d& pose, mp::quantity<mp::rad / mp::m> curvature,
      mp::quantity<mp::m / mp::s> velocity) const override {
    auto xVelocity = velocity * pose.Rotation().Cos();
    auto yVelocity = velocity * pose.Rotation().Sin();
    auto wheelSpeeds =
        m_kinematics.ToWheelSpeeds({xVelocity, yVelocity, velocity * curvature})
            .Desaturate(m_maxSpeed);

    auto normSpeeds = m_kinematics.ToChassisSpeeds(wheelSpeeds);

    return mp::hypot(normSpeeds.vx, normSpeeds.vy);
  }

  MinMax MinMaxAcceleration(const Pose2d& pose,
                            mp::quantity<mp::rad / mp::m> curvature,
                            mp::quantity<mp::m / mp::s> speed) const override {
    return {};
  }

 private:
  MecanumDriveKinematics m_kinematics;
  mp::quantity<mp::m / mp::s> m_maxSpeed;
};
}  // namespace frc
