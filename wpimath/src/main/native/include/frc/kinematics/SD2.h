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

namespace sd2 {

/**
 * Discretizes a continuous-time chassis speed with saturation constraints.
 *
 * @param continuousSpeeds The continuous speeds.
 * @param dt The duration of the timestep the speeds should be applied for.
 * @param maxModuleSpeed The max speed that a module can reach.
 * @param modules The positions of the swerve modules.
 * @param debug Parameter to activate debug prints. Should be false except in
 * testing, and will be removed.
 * @param v3 Parameter to select the v3 implementation version. Should be true
 * except when comparing with v2, and will be removed.
 * @return The discretized and desaturated chassis speeds.
 * @see ChassisSpeeds::Discretize
 * @see SwerveDriveKinematics::DesaturateWheelSpeeds
 */
WPILIB_DLLEXPORT frc::ChassisSpeeds DesaturatedDiscretize(
    const frc::ChassisSpeeds& continuousSpeeds, units::second_t dt,
    units::meters_per_second_t maxModuleSpeed,
    std::span<const frc::Translation2d> modules, bool debug = false,
    bool v3 = true);

}  // namespace sd2
