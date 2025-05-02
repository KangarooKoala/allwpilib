// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/MathExtras.h>
#include <wpi/SymbolExports.h>

#include "frc/units.h"

namespace frc {
/**
 * Represents the wheel positions for a differential drive drivetrain.
 */
struct WPILIB_DLLEXPORT DifferentialDriveWheelPositions {
  /**
   * Distance driven by the left side.
   */
  mp::quantity<mp::m> left = 0.0 * mp::m;

  /**
   * Distance driven by the right side.
   */
  mp::quantity<mp::m> right = 0.0 * mp::m;

  /**
   * Checks equality between this DifferentialDriveWheelPositions and another
   * object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  constexpr bool operator==(
      const DifferentialDriveWheelPositions& other) const = default;

  constexpr DifferentialDriveWheelPositions Interpolate(
      const DifferentialDriveWheelPositions& endValue, double t) const {
    return {wpi::Lerp(left, endValue.left, t),
            wpi::Lerp(right, endValue.right, t)};
  }
};
}  // namespace frc

#include "frc/kinematics/proto/DifferentialDriveWheelPositionsProto.h"
#include "frc/kinematics/struct/DifferentialDriveWheelPositionsStruct.h"
