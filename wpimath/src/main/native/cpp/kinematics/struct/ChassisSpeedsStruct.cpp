// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/kinematics/struct/ChassisSpeedsStruct.h"

namespace {
constexpr size_t kVxOff = 0;
constexpr size_t kVyOff = kVxOff + 8;
constexpr size_t kOmegaOff = kVyOff + 8;
}  // namespace

using StructType = wpi::Struct<frc::ChassisSpeeds>;

frc::ChassisSpeeds StructType::Unpack(std::span<const uint8_t> data) {
  return frc::ChassisSpeeds{
      wpi::UnpackStruct<double, kVxOff>(data) * mp::m / mp::s,
      wpi::UnpackStruct<double, kVyOff>(data) * mp::m / mp::s,
      wpi::UnpackStruct<double, kOmegaOff>(data) * mp::rad / mp::s,
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const frc::ChassisSpeeds& value) {
  wpi::PackStruct<kVxOff>(data, mp::value(value.vx));
  wpi::PackStruct<kVyOff>(data, mp::value(value.vy));
  wpi::PackStruct<kOmegaOff>(data, mp::value(value.omega));
}
