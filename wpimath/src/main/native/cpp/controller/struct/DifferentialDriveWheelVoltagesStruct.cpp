// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/controller/struct/DifferentialDriveWheelVoltagesStruct.h"

namespace {
constexpr size_t kLeftOff = 0;
constexpr size_t kRightOff = kLeftOff + 8;
}  // namespace

using StructType = wpi::Struct<frc::DifferentialDriveWheelVoltages>;

frc::DifferentialDriveWheelVoltages StructType::Unpack(
    std::span<const uint8_t> data) {
  return frc::DifferentialDriveWheelVoltages{
      wpi::UnpackStruct<double, kLeftOff>(data) * units::volt,
      wpi::UnpackStruct<double, kRightOff>(data) * units::volt,
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const frc::DifferentialDriveWheelVoltages& value) {
  wpi::PackStruct<kLeftOff>(data, value.left.value());
  wpi::PackStruct<kRightOff>(data, value.right.value());
}
