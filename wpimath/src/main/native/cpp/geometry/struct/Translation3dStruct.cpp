// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/struct/Translation3dStruct.h"

namespace {
constexpr size_t kXOff = 0;
constexpr size_t kYOff = kXOff + 8;
constexpr size_t kZOff = kYOff + 8;
}  // namespace

using StructType = wpi::Struct<frc::Translation3d>;

frc::Translation3d StructType::Unpack(std::span<const uint8_t> data) {
  return frc::Translation3d{
      wpi::UnpackStruct<double, kXOff>(data) * mp::m,
      wpi::UnpackStruct<double, kYOff>(data) * mp::m,
      wpi::UnpackStruct<double, kZOff>(data) * mp::m,
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const frc::Translation3d& value) {
  wpi::PackStruct<kXOff>(data, mp::value(value.X()));
  wpi::PackStruct<kYOff>(data, mp::value(value.Y()));
  wpi::PackStruct<kZOff>(data, mp::value(value.Z()));
}
