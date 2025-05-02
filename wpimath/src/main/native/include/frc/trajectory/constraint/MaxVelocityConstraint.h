// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>

#include "frc/trajectory/constraint/TrajectoryConstraint.h"
#include "frc/units.h"

namespace frc {

/**
 * Represents a constraint that enforces a max velocity. This can be composed
 * with the EllipticalRegionConstraint or RectangularRegionConstraint to enforce
 * a max velocity within a region.
 */
class WPILIB_DLLEXPORT MaxVelocityConstraint : public TrajectoryConstraint {
 public:
  /**
   * Constructs a new MaxVelocityConstraint.
   *
   * @param maxVelocity The max velocity.
   */
  constexpr explicit MaxVelocityConstraint(
      mp::quantity<mp::m / mp::s> maxVelocity)
      : m_maxVelocity(mp::abs(maxVelocity)) {}

  constexpr mp::quantity<mp::m / mp::s> MaxVelocity(
      const Pose2d& pose, mp::quantity<mp::rad / mp::m> curvature,
      mp::quantity<mp::m / mp::s> velocity) const override {
    return m_maxVelocity;
  }

  constexpr MinMax MinMaxAcceleration(
      const Pose2d& pose, mp::quantity<mp::rad / mp::m> curvature,
      mp::quantity<mp::m / mp::s> speed) const override {
    return {};
  }

 private:
  mp::quantity<mp::m / mp::s> m_maxVelocity;
};

}  // namespace frc
