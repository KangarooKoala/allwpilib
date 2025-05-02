// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <type_traits>

#include <wpi/SymbolExports.h>

#include "frc/geometry/Twist2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/DifferentialDriveWheelPositions.h"
#include "frc/kinematics/DifferentialDriveWheelSpeeds.h"
#include "frc/kinematics/Kinematics.h"
#include "frc/units.h"
#include "wpimath/MathShared.h"

namespace frc {
/**
 * Helper class that converts a chassis velocity (dx and dtheta components) to
 * left and right wheel velocities for a differential drive.
 *
 * Inverse kinematics converts a desired chassis speed into left and right
 * velocity components whereas forward kinematics converts left and right
 * component velocities into a linear and angular chassis speed.
 */
class WPILIB_DLLEXPORT DifferentialDriveKinematics
    : public Kinematics<DifferentialDriveWheelSpeeds,
                        DifferentialDriveWheelPositions> {
 public:
  /**
   * Constructs a differential drive kinematics object.
   *
   * @param trackwidth The trackwidth of the drivetrain. Theoretically, this is
   * the distance between the left wheels and right wheels. However, the
   * empirical value may be larger than the physical measured value due to
   * scrubbing effects.
   */
  constexpr explicit DifferentialDriveKinematics(mp::quantity<mp::m> trackwidth)
      : trackwidth(trackwidth) {
    if (!std::is_constant_evaluated()) {
      wpi::math::MathSharedStore::ReportUsage("DifferentialDriveKinematics",
                                              "");
    }
  }

  /**
   * Returns a chassis speed from left and right component velocities using
   * forward kinematics.
   *
   * @param wheelSpeeds The left and right velocities.
   * @return The chassis speed.
   */
  constexpr ChassisSpeeds ToChassisSpeeds(
      const DifferentialDriveWheelSpeeds& wheelSpeeds) const override {
    return {
        (wheelSpeeds.left + wheelSpeeds.right) / 2.0, 0.0 * mp::m / mp::s,
        (wheelSpeeds.right - wheelSpeeds.left) / trackwidth * 1.0 * mp::rad};
  }

  /**
   * Returns left and right component velocities from a chassis speed using
   * inverse kinematics.
   *
   * @param chassisSpeeds The linear and angular (dx and dtheta) components that
   * represent the chassis' speed.
   * @return The left and right velocities.
   */
  constexpr DifferentialDriveWheelSpeeds ToWheelSpeeds(
      const ChassisSpeeds& chassisSpeeds) const override {
    return {chassisSpeeds.vx -
                trackwidth / 2 * chassisSpeeds.omega / (1.0 * mp::rad),
            chassisSpeeds.vx +
                trackwidth / 2 * chassisSpeeds.omega / (1.0 * mp::rad)};
  }

  /**
   * Returns a twist from left and right distance deltas using
   * forward kinematics.
   *
   * @param leftDistance The distance measured by the left encoder.
   * @param rightDistance The distance measured by the right encoder.
   * @return The resulting Twist2d.
   */
  constexpr Twist2d ToTwist2d(const mp::quantity<mp::m> leftDistance,
                              const mp::quantity<mp::m> rightDistance) const {
    return {(leftDistance + rightDistance) / 2, 0.0 * mp::m,
            (rightDistance - leftDistance) / trackwidth * 1.0 * mp::rad};
  }

  constexpr Twist2d ToTwist2d(
      const DifferentialDriveWheelPositions& start,
      const DifferentialDriveWheelPositions& end) const override {
    return ToTwist2d(end.left - start.left, end.right - start.right);
  }

  constexpr DifferentialDriveWheelPositions Interpolate(
      const DifferentialDriveWheelPositions& start,
      const DifferentialDriveWheelPositions& end, double t) const override {
    return start.Interpolate(end, t);
  }

  /// Differential drive trackwidth.
  mp::quantity<mp::m> trackwidth;
};
}  // namespace frc

#include "frc/kinematics/proto/DifferentialDriveKinematicsProto.h"
#include "frc/kinematics/struct/DifferentialDriveKinematicsStruct.h"
