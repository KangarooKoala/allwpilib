// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>

#include "frc/units.h"

namespace frc {

/**
 * A change in distance along a 3D arc since the last pose update. We can use
 * ideas from differential calculus to create new Pose3ds from a Twist3d and
 * vice versa.
 *
 * A Twist can be used to represent a difference between two poses.
 */
struct WPILIB_DLLEXPORT Twist3d {
  /**
   * Linear "dx" component
   */
  mp::quantity<mp::m> dx = 0.0 * mp::m;

  /**
   * Linear "dy" component
   */
  mp::quantity<mp::m> dy = 0.0 * mp::m;

  /**
   * Linear "dz" component
   */
  mp::quantity<mp::m> dz = 0.0 * mp::m;

  /**
   * Rotation vector x component.
   */
  mp::quantity<mp::rad> rx = 0.0 * mp::rad;

  /**
   * Rotation vector y component.
   */
  mp::quantity<mp::rad> ry = 0.0 * mp::rad;

  /**
   * Rotation vector z component.
   */
  mp::quantity<mp::rad> rz = 0.0 * mp::rad;

  /**
   * Checks equality between this Twist3d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  constexpr bool operator==(const Twist3d& other) const {
    return mp::abs(dx - other.dx) < 1E-9 * mp::m &&
           mp::abs(dy - other.dy) < 1E-9 * mp::m &&
           mp::abs(dz - other.dz) < 1E-9 * mp::m &&
           mp::abs(rx - other.rx) < 1E-9 * mp::rad &&
           mp::abs(ry - other.ry) < 1E-9 * mp::rad &&
           mp::abs(rz - other.rz) < 1E-9 * mp::rad;
  }

  /**
   * Scale this by a given factor.
   *
   * @param factor The factor by which to scale.
   * @return The scaled Twist3d.
   */
  constexpr Twist3d operator*(double factor) const {
    return Twist3d{dx * factor, dy * factor, dz * factor,
                   rx * factor, ry * factor, rz * factor};
  }
};

}  // namespace frc

#include "frc/geometry/proto/Twist3dProto.h"
#include "frc/geometry/struct/Twist3dStruct.h"
