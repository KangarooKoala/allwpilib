// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/controller/DifferentialDriveFeedforward.h"

#include <Eigen/Core>

#include "frc/controller/LinearPlantInversionFeedforward.h"
#include "frc/units.h"

using namespace frc;

DifferentialDriveWheelVoltages DifferentialDriveFeedforward::Calculate(
    mp::quantity<mp::m / mp::s> currentLeftVelocity,
    mp::quantity<mp::m / mp::s> nextLeftVelocity,
    mp::quantity<mp::m / mp::s> currentRightVelocity,
    mp::quantity<mp::m / mp::s> nextRightVelocity, mp::quantity<mp::s> dt) {
  frc::LinearPlantInversionFeedforward<2, 2> feedforward{m_plant, dt};

  Eigen::Vector2d r{mp::value(currentLeftVelocity),
                    mp::value(currentRightVelocity)};
  Eigen::Vector2d nextR{mp::value(nextLeftVelocity),
                        mp::value(nextRightVelocity)};
  auto u = feedforward.Calculate(r, nextR);
  return {u(0) * mp::V, u(1) * mp::V};
}
