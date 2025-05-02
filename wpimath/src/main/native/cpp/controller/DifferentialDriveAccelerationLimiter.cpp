// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/controller/DifferentialDriveAccelerationLimiter.h"

#include <Eigen/QR>

#include "frc/units.h"

using namespace frc;

DifferentialDriveWheelVoltages DifferentialDriveAccelerationLimiter::Calculate(
    mp::quantity<mp::m / mp::s> leftVelocity,
    mp::quantity<mp::m / mp::s> rightVelocity, mp::quantity<mp::V> leftVoltage,
    mp::quantity<mp::V> rightVoltage) {
  Vectord<2> u{mp::value(leftVoltage), mp::value(rightVoltage)};

  // Find unconstrained wheel accelerations
  Vectord<2> x{mp::value(leftVelocity), mp::value(rightVelocity)};
  Vectord<2> dxdt = m_system.A() * x + m_system.B() * u;

  // Convert from wheel accelerations to linear and angular accelerations
  //
  // a = (dxdt(0) + dx/dt(1)) / 2
  //   = 0.5 dxdt(0) + 0.5 dxdt(1)
  //
  // α = (dxdt(1) - dxdt(0)) / trackwidth
  //   = -1/trackwidth dxdt(0) + 1/trackwidth dxdt(1)
  //
  // [a] = [          0.5           0.5][dxdt(0)]
  // [α]   [-1/trackwidth  1/trackwidth][dxdt(1)]
  //
  // accels = M dxdt where M = [0.5, 0.5; -1/trackwidth, 1/trackwidth]
  Matrixd<2, 2> M{
      {0.5, 0.5},
      {-1.0 / mp::value(m_trackwidth), 1.0 / mp::value(m_trackwidth)}};
  Vectord<2> accels = M * dxdt;

  // Constrain the linear and angular accelerations
  if (accels(0) > mp::value(m_maxLinearAccel)) {
    accels(0) = mp::value(m_maxLinearAccel);
  } else if (accels(0) < mp::value(m_minLinearAccel)) {
    accels(0) = mp::value(m_minLinearAccel);
  }
  if (accels(1) > mp::value(m_maxAngularAccel)) {
    accels(1) = mp::value(m_maxAngularAccel);
  } else if (accels(1) < -mp::value(m_maxAngularAccel)) {
    accels(1) = -mp::value(m_maxAngularAccel);
  }

  // Convert the constrained linear and angular accelerations back to wheel
  // accelerations
  dxdt = M.householderQr().solve(accels);

  // Find voltages for the given wheel accelerations
  //
  // dx/dt = Ax + Bu
  // u = B⁻¹(dx/dt - Ax)
  u = m_system.B().householderQr().solve(dxdt - m_system.A() * x);

  return {u(0) * mp::V, u(1) * mp::V};
}
