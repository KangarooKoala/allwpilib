// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <array>
#include <cmath>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "frc/estimator/KalmanFilter.h"
#include "frc/system/plant/DCMotor.h"
#include "frc/system/plant/LinearSystemId.h"
#include "frc/units.h"

TEST(KalmanFilterTest, Flywheel) {
  auto motor = frc::DCMotor::NEO();
  auto flywheel =
      frc::LinearSystemId::FlywheelSystem(motor, 1.0 * mp::kg * mp::m2, 1.0);
  frc::KalmanFilter<1, 1, 1> kf{flywheel, {1}, {1}, 5.0 * mp::ms};
}
