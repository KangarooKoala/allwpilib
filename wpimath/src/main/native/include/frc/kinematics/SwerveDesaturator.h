// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <span>

#include <wpi/SymbolExports.h>

#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "units/time.h"
#include "units/velocity.h"

namespace frc {

/**
 * Class for swerve desaturation.
 *
 * TODO Should this be a namespace instead of a class?
 * TODO Should the methods be directly in frc?
 */
class WPILIB_DLLEXPORT SwerveDesaturator {
 public:
  /**
   * Discretizes a continuous-time chassis speed with saturation constraints.
   *
   * @param continuousSpeeds The continuous speeds.
   * @param dt The duration of the timestep the speeds should be applied for.
   * @param maxModuleSpeed The max speed that a module can reach.
   * @param modules The positions of the swerve modules.
   * @param debug Parameter to activate debug prints. Should be false except in
   * testing, and will be removed.
   * @return The discretized and desaturated chassis speeds.
   * @see ChassisSpeeds::Discretize
   * @see SwerveDriveKinematics::DesaturateWheelSpeeds
   */
  static ChassisSpeeds DesaturatedDiscretize(
      const ChassisSpeeds& continuousSpeeds, units::second_t dt,
      units::meters_per_second_t maxModuleSpeed,
      std::span<const Translation2d> modules, bool debug = false);
};

}  // namespace frc
