// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>

#include "frc/trajectory/constraint/TrajectoryConstraint.h"
#include "frc/units.h"

namespace frc {

/**
 * A constraint on the maximum absolute centripetal acceleration allowed when
 * traversing a trajectory. The centripetal acceleration of a robot is defined
 * as the velocity squared divided by the radius of curvature.
 *
 * Effectively, limiting the maximum centripetal acceleration will cause the
 * robot to slow down around tight turns, making it easier to track trajectories
 * with sharp turns.
 */
class WPILIB_DLLEXPORT CentripetalAccelerationConstraint
    : public TrajectoryConstraint {
 public:
  constexpr explicit CentripetalAccelerationConstraint(
      mp::quantity<mp::m / mp::s2> maxCentripetalAcceleration)
      : m_maxCentripetalAcceleration(maxCentripetalAcceleration) {}

  constexpr mp::quantity<mp::m / mp::s> MaxVelocity(
      const Pose2d& pose, mp::quantity<mp::rad / mp::m> curvature,
      mp::quantity<mp::m / mp::s> velocity) const override {
    // ac = v²/r
    // k (curvature) = 1/r

    // therefore, ac = v²k
    // ac/k = v²
    // v = √(ac/k)

    // We have to multiply by rad here to get the units to cancel out nicely.
    // The units library defines a unit for radians although it is technically
    // unitless.
    return mp::sqrt(m_maxCentripetalAcceleration / mp::abs(curvature) *
                    mp::rad);
  }

  constexpr MinMax MinMaxAcceleration(
      const Pose2d& pose, mp::quantity<mp::rad / mp::m> curvature,
      mp::quantity<mp::m / mp::s> speed) const override {
    // The acceleration of the robot has no impact on the centripetal
    // acceleration of the robot.
    return {};
  }

 private:
  mp::quantity<mp::m / mp::s2> m_maxCentripetalAcceleration;
};
}  // namespace frc
