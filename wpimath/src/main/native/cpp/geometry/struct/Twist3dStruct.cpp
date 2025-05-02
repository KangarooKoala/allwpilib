// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/struct/Twist3dStruct.h"

namespace {
constexpr size_t kDxOff = 0;
constexpr size_t kDyOff = kDxOff + 8;
constexpr size_t kDzOff = kDyOff + 8;
constexpr size_t kRxOff = kDzOff + 8;
constexpr size_t kRyOff = kRxOff + 8;
constexpr size_t kRzOff = kRyOff + 8;
}  // namespace

using StructType = wpi::Struct<frc::Twist3d>;

frc::Twist3d StructType::Unpack(std::span<const uint8_t> data) {
  return frc::Twist3d{
      wpi::UnpackStruct<double, kDxOff>(data) * mp::m,
      wpi::UnpackStruct<double, kDyOff>(data) * mp::m,
      wpi::UnpackStruct<double, kDzOff>(data) * mp::m,
      wpi::UnpackStruct<double, kRxOff>(data) * mp::rad,
      wpi::UnpackStruct<double, kRyOff>(data) * mp::rad,
      wpi::UnpackStruct<double, kRzOff>(data) * mp::rad,
  };
}

void StructType::Pack(std::span<uint8_t> data, const frc::Twist3d& value) {
  wpi::PackStruct<kDxOff>(data, mp::value(value.dx));
  wpi::PackStruct<kDyOff>(data, mp::value(value.dy));
  wpi::PackStruct<kDzOff>(data, mp::value(value.dz));
  wpi::PackStruct<kRxOff>(data, mp::value(value.rx));
  wpi::PackStruct<kRyOff>(data, mp::value(value.ry));
  wpi::PackStruct<kRzOff>(data, mp::value(value.rz));
}
