// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <utility>

#include <Eigen/Core>
#include <wpi/SymbolExports.h>

#include "frc/controller/DifferentialDriveWheelVoltages.h"
#include "frc/system/LinearSystem.h"
#include "frc/units.h"

namespace frc {

/**
 * Filters the provided voltages to limit a differential drive's linear and
 * angular acceleration.
 *
 * The differential drive model can be created via the functions in
 * LinearSystemId.
 */
class WPILIB_DLLEXPORT DifferentialDriveAccelerationLimiter {
 public:
  /**
   * Constructs a DifferentialDriveAccelerationLimiter.
   *
   * @param system The differential drive dynamics.
   * @param trackwidth The distance between the differential drive's left and
   *                   right wheels.
   * @param maxLinearAccel The maximum linear acceleration.
   * @param maxAngularAccel The maximum angular acceleration.
   */
  DifferentialDriveAccelerationLimiter(
      LinearSystem<2, 2, 2> system, mp::quantity<mp::m> trackwidth,
      mp::quantity<mp::m / mp::s2> maxLinearAccel,
      mp::quantity<mp::rad / mp::s2> maxAngularAccel)
      : DifferentialDriveAccelerationLimiter(system, trackwidth,
                                             -maxLinearAccel, maxLinearAccel,
                                             maxAngularAccel) {}

  /**
   * Constructs a DifferentialDriveAccelerationLimiter.
   *
   * @param system The differential drive dynamics.
   * @param trackwidth The distance between the differential drive's left and
   *                   right wheels.
   * @param minLinearAccel The minimum (most negative) linear acceleration.
   * @param maxLinearAccel The maximum (most positive) linear acceleration.
   * @param maxAngularAccel The maximum angular acceleration.
   * @throws std::invalid_argument if minimum linear acceleration is greater
   * than maximum linear acceleration
   */
  DifferentialDriveAccelerationLimiter(
      LinearSystem<2, 2, 2> system, mp::quantity<mp::m> trackwidth,
      mp::quantity<mp::m / mp::s2> minLinearAccel,
      mp::quantity<mp::m / mp::s2> maxLinearAccel,
      mp::quantity<mp::rad / mp::s2> maxAngularAccel)
      : m_system{std::move(system)},
        m_trackwidth{trackwidth},
        m_minLinearAccel{minLinearAccel},
        m_maxLinearAccel{maxLinearAccel},
        m_maxAngularAccel{maxAngularAccel} {
    if (minLinearAccel > maxLinearAccel) {
      throw std::invalid_argument(
          "maxLinearAccel must be greater than minLinearAccel");
    }
  }

  /**
   * Returns the next voltage pair subject to acceleration constraints.
   *
   * @param leftVelocity The left wheel velocity.
   * @param rightVelocity The right wheel velocity.
   * @param leftVoltage The unconstrained left motor voltage.
   * @param rightVoltage The unconstrained right motor voltage.
   * @return The constrained wheel voltages.
   */
  DifferentialDriveWheelVoltages Calculate(
      mp::quantity<mp::m / mp::s> leftVelocity,
      mp::quantity<mp::m / mp::s> rightVelocity,
      mp::quantity<mp::V> leftVoltage, mp::quantity<mp::V> rightVoltage);

 private:
  LinearSystem<2, 2, 2> m_system;
  mp::quantity<mp::m> m_trackwidth;
  mp::quantity<mp::m / mp::s2> m_minLinearAccel;
  mp::quantity<mp::m / mp::s2> m_maxLinearAccel;
  mp::quantity<mp::rad / mp::s2> m_maxAngularAccel;
};

}  // namespace frc
