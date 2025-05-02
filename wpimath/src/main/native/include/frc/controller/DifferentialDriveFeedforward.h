// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>

#include "frc/controller/DifferentialDriveWheelVoltages.h"
#include "frc/system/LinearSystem.h"
#include "frc/system/plant/LinearSystemId.h"
#include "frc/units.h"

namespace frc {
/**
 * A helper class which computes the feedforward outputs for a differential
 * drive drivetrain.
 */
class WPILIB_DLLEXPORT DifferentialDriveFeedforward {
  frc::LinearSystem<2, 2, 2> m_plant;

 public:
  /**
   * Creates a new DifferentialDriveFeedforward with the specified parameters.
   *
   * @param kVLinear The linear velocity gain in volts per (meters per second).
   * @param kALinear The linear acceleration gain in volts per (meters per
   * second squared).
   * @param kVAngular The angular velocity gain in volts per (radians per
   * second).
   * @param kAAngular The angular acceleration gain in volts per (radians per
   * second squared).
   * @param trackwidth The distance between the differential drive's left and
   * right wheels, in meters.
   */
  constexpr DifferentialDriveFeedforward(
      mp::quantity<mp::V / (mp::m / mp::s)> kVLinear,
      mp::quantity<mp::V / (mp::m / mp::s2)> kALinear,
      mp::quantity<mp::V / (mp::rad / mp::s)> kVAngular,
      mp::quantity<mp::V / (mp::rad / mp::s2)> kAAngular,
      mp::quantity<mp::m> trackwidth)
      // See LinearSystemId::IdentifyDrivetrainSystem(
      // mp::quantity<mp::V / (mp::m / mp::s)>,
      // mp::quantity<mp::V / (mp::m / mp::s2)>,
      // mp::quantity<mp::V / (mp::rad / mp::s)>,
      // mp::quantity<mp::V / (mp::rad / mp::s2)>)
      : DifferentialDriveFeedforward{
            kVLinear, kALinear, kVAngular * 2.0 / trackwidth * 1.0 * mp::rad,
            kAAngular * 2.0 / trackwidth * 1.0 * mp::rad} {}

  /**
   * Creates a new DifferentialDriveFeedforward with the specified parameters.
   *
   * @param kVLinear The linear velocity gain in volts per (meters per second).
   * @param kALinear The linear acceleration gain in volts per (meters per
   * second squared).
   * @param kVAngular The angular velocity gain in volts per (meters per
   * second).
   * @param kAAngular The angular acceleration gain in volts per (meters per
   * second squared).
   */
  constexpr DifferentialDriveFeedforward(
      mp::quantity<mp::V / (mp::m / mp::s)> kVLinear,
      mp::quantity<mp::V / (mp::m / mp::s2)> kALinear,
      mp::quantity<mp::V / (mp::m / mp::s)> kVAngular,
      mp::quantity<mp::V / (mp::m / mp::s2)> kAAngular)
      : m_plant{frc::LinearSystemId::IdentifyDrivetrainSystem(
            kVLinear, kALinear, kVAngular, kAAngular)},
        m_kVLinear{kVLinear},
        m_kALinear{kALinear},
        m_kVAngular{kVAngular},
        m_kAAngular{kAAngular} {}

  /**
   * Calculates the differential drive feedforward inputs given velocity
   * setpoints.
   *
   * @param currentLeftVelocity The current left velocity of the differential
   * drive in meters/second.
   * @param nextLeftVelocity The next left velocity of the differential drive in
   * meters/second.
   * @param currentRightVelocity The current right velocity of the differential
   * drive in meters/second.
   * @param nextRightVelocity The next right velocity of the differential drive
   * in meters/second.
   * @param dt Discretization timestep.
   */
  DifferentialDriveWheelVoltages Calculate(
      mp::quantity<mp::m / mp::s> currentLeftVelocity,
      mp::quantity<mp::m / mp::s> nextLeftVelocity,
      mp::quantity<mp::m / mp::s> currentRightVelocity,
      mp::quantity<mp::m / mp::s> nextRightVelocity, mp::quantity<mp::s> dt);

  mp::quantity<mp::V / (mp::m / mp::s)> m_kVLinear;
  mp::quantity<mp::V / (mp::m / mp::s2)> m_kALinear;
  mp::quantity<mp::V / (mp::m / mp::s)> m_kVAngular;
  mp::quantity<mp::V / (mp::m / mp::s2)> m_kAAngular;
};
}  // namespace frc

#include "frc/controller/proto/DifferentialDriveFeedforwardProto.h"
#include "frc/controller/struct/DifferentialDriveFeedforwardStruct.h"
