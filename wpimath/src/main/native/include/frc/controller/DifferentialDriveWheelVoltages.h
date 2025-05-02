// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/units.h"

namespace frc {

/**
 * Motor voltages for a differential drive.
 */
struct DifferentialDriveWheelVoltages {
  /// Left wheel voltage.
  mp::quantity<mp::V> left = 0.0 * mp::V;

  /// Right wheel voltage.
  mp::quantity<mp::V> right = 0.0 * mp::V;
};

}  // namespace frc

#include "frc/controller/proto/DifferentialDriveWheelVoltagesProto.h"
#include "frc/controller/struct/DifferentialDriveWheelVoltagesStruct.h"
