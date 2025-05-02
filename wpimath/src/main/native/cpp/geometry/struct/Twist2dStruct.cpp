// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/struct/Twist2dStruct.h"

namespace {
constexpr size_t kDxOff = 0;
constexpr size_t kDyOff = kDxOff + 8;
constexpr size_t kDthetaOff = kDyOff + 8;
}  // namespace

using StructType = wpi::Struct<frc::Twist2d>;

frc::Twist2d StructType::Unpack(std::span<const uint8_t> data) {
  return frc::Twist2d{
      wpi::UnpackStruct<double, kDxOff>(data) * mp::m,
      wpi::UnpackStruct<double, kDyOff>(data) * mp::m,
      wpi::UnpackStruct<double, kDthetaOff>(data) * mp::rad,
  };
}

void StructType::Pack(std::span<uint8_t> data, const frc::Twist2d& value) {
  wpi::PackStruct<kDxOff>(data, mp::value(value.dx));
  wpi::PackStruct<kDyOff>(data, mp::value(value.dy));
  wpi::PackStruct<kDthetaOff>(data, mp::value(value.dtheta));
}
