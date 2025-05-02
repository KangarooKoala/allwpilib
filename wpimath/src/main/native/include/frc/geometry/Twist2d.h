// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>

#include "frc/units.h"

namespace frc {

/**
 * A change in distance along a 2D arc since the last pose update. We can use
 * ideas from differential calculus to create new Pose2ds from a Twist2d and
 * vice versa.
 *
 * A Twist can be used to represent a difference between two poses.
 */
struct WPILIB_DLLEXPORT Twist2d {
  /**
   * Linear "dx" component
   */
  mp::quantity<mp::m> dx = 0.0 * mp::m;

  /**
   * Linear "dy" component
   */
  mp::quantity<mp::m> dy = 0.0 * mp::m;

  /**
   * Angular "dtheta" component (radians)
   */
  mp::quantity<mp::rad> dtheta = 0.0 * mp::rad;

  /**
   * Checks equality between this Twist2d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  constexpr bool operator==(const Twist2d& other) const {
    return mp::abs(dx - other.dx) < 1E-9 * mp::m &&
           mp::abs(dy - other.dy) < 1E-9 * mp::m &&
           mp::abs(dtheta - other.dtheta) < 1E-9 * mp::rad;
  }

  /**
   * Scale this by a given factor.
   *
   * @param factor The factor by which to scale.
   * @return The scaled Twist2d.
   */
  constexpr Twist2d operator*(double factor) const {
    return Twist2d{dx * factor, dy * factor, dtheta * factor};
  }
};

}  // namespace frc

#include "frc/geometry/proto/Twist2dProto.h"
#include "frc/geometry/struct/Twist2dStruct.h"
