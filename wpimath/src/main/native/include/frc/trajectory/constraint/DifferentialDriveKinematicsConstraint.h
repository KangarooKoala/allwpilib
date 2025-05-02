// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <utility>

#include <wpi/SymbolExports.h>

#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/trajectory/constraint/TrajectoryConstraint.h"
#include "frc/units.h"

namespace frc {
/**
 * A class that enforces constraints on the differential drive kinematics.
 * This can be used to ensure that the trajectory is constructed so that the
 * commanded velocities for both sides of the drivetrain stay below a certain
 * limit.
 */
class WPILIB_DLLEXPORT DifferentialDriveKinematicsConstraint
    : public TrajectoryConstraint {
 public:
  constexpr DifferentialDriveKinematicsConstraint(
      DifferentialDriveKinematics kinematics,
      mp::quantity<mp::m / mp::s> maxSpeed)
      : m_kinematics(std::move(kinematics)), m_maxSpeed(maxSpeed) {}

  constexpr mp::quantity<mp::m / mp::s> MaxVelocity(
      const Pose2d& pose, mp::quantity<mp::rad / mp::m> curvature,
      mp::quantity<mp::m / mp::s> velocity) const override {
    auto wheelSpeeds = m_kinematics.ToWheelSpeeds(
        {velocity, 0.0 * mp::m / mp::s, velocity * curvature});
    wheelSpeeds.Desaturate(m_maxSpeed);

    return m_kinematics.ToChassisSpeeds(wheelSpeeds).vx;
  }

  constexpr MinMax MinMaxAcceleration(
      const Pose2d& pose, mp::quantity<mp::rad / mp::m> curvature,
      mp::quantity<mp::m / mp::s> speed) const override {
    return {};
  }

 private:
  DifferentialDriveKinematics m_kinematics;
  mp::quantity<mp::m / mp::s> m_maxSpeed;
};
}  // namespace frc
