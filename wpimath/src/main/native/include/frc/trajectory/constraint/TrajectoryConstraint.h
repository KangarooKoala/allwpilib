// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <limits>

#include <wpi/SymbolExports.h>

#include "frc/geometry/Pose2d.h"
#include "frc/units.h"

namespace frc {
/**
 * An interface for defining user-defined velocity and acceleration constraints
 * while generating trajectories.
 */
class WPILIB_DLLEXPORT TrajectoryConstraint {
 public:
  constexpr TrajectoryConstraint() = default;

  constexpr TrajectoryConstraint(const TrajectoryConstraint&) = default;
  constexpr TrajectoryConstraint& operator=(const TrajectoryConstraint&) =
      default;

  constexpr TrajectoryConstraint(TrajectoryConstraint&&) = default;
  constexpr TrajectoryConstraint& operator=(TrajectoryConstraint&&) = default;

  constexpr virtual ~TrajectoryConstraint() = default;

  /**
   * Represents a minimum and maximum acceleration.
   */
  struct MinMax {
    /**
     * The minimum acceleration.
     */
    mp::quantity<mp::m / mp::s2> minAcceleration =
        -std::numeric_limits<double>::max() * mp::m / mp::s2;

    /**
     * The maximum acceleration.
     */
    mp::quantity<mp::m / mp::s2> maxAcceleration =
        std::numeric_limits<double>::max() * mp::m / mp::s2;
  };

  /**
   * Returns the max velocity given the current pose and curvature.
   *
   * @param pose The pose at the current point in the trajectory.
   * @param curvature The curvature at the current point in the trajectory.
   * @param velocity The velocity at the current point in the trajectory before
   *                                constraints are applied.
   *
   * @return The absolute maximum velocity.
   */
  constexpr virtual mp::quantity<mp::m / mp::s> MaxVelocity(
      const Pose2d& pose, mp::quantity<mp::rad / mp::m> curvature,
      mp::quantity<mp::m / mp::s> velocity) const = 0;

  /**
   * Returns the minimum and maximum allowable acceleration for the trajectory
   * given pose, curvature, and speed.
   *
   * @param pose The pose at the current point in the trajectory.
   * @param curvature The curvature at the current point in the trajectory.
   * @param speed The speed at the current point in the trajectory.
   *
   * @return The min and max acceleration bounds.
   */
  constexpr virtual MinMax MinMaxAcceleration(
      const Pose2d& pose, mp::quantity<mp::rad / mp::m> curvature,
      mp::quantity<mp::m / mp::s> speed) const = 0;
};
}  // namespace frc
