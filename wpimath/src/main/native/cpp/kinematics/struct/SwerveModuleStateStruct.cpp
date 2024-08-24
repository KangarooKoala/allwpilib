// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/kinematics/struct/SwerveModuleStateStruct.h"

namespace {
constexpr size_t kSpeedOff = 0;
constexpr size_t kAngleOff = kSpeedOff + 8;
}  // namespace

using StructType = wpi::Struct<frc::SwerveModuleState>;

frc::SwerveModuleState StructType::Unpack(std::span<const uint8_t> data) {
  return frc::SwerveModuleState{
      wpi::UnpackStruct<double, kSpeedOff>(data) * units::meters_per_second,
      wpi::UnpackStruct<frc::Rotation2d, kAngleOff>(data),
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const frc::SwerveModuleState& value) {
  wpi::PackStruct<kSpeedOff>(data, value.speed.value());
  wpi::PackStruct<kAngleOff>(data, value.angle);
}
