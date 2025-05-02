// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/kinematics/struct/SwerveModulePositionStruct.h"

namespace {
constexpr size_t kDistanceOff = 0;
constexpr size_t kAngleOff = kDistanceOff + 8;
}  // namespace

using StructType = wpi::Struct<frc::SwerveModulePosition>;

frc::SwerveModulePosition StructType::Unpack(std::span<const uint8_t> data) {
  return frc::SwerveModulePosition{
      wpi::UnpackStruct<double, kDistanceOff>(data) * mp::m,
      wpi::UnpackStruct<frc::Rotation2d, kAngleOff>(data),
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const frc::SwerveModulePosition& value) {
  wpi::PackStruct<kDistanceOff>(data, mp::value(value.distance));
  wpi::PackStruct<kAngleOff>(data, value.angle);
}
