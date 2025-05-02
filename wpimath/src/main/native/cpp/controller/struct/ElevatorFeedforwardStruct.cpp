// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/controller/struct/ElevatorFeedforwardStruct.h"

namespace {
constexpr size_t kKsOff = 0;
constexpr size_t kKgOff = kKsOff + 8;
constexpr size_t kKvOff = kKgOff + 8;
constexpr size_t kKaOff = kKvOff + 8;
}  // namespace

using StructType = wpi::Struct<frc::ElevatorFeedforward>;

frc::ElevatorFeedforward StructType::Unpack(std::span<const uint8_t> data) {
  return frc::ElevatorFeedforward{
      wpi::UnpackStruct<double, kKsOff>(data) * mp::V,
      wpi::UnpackStruct<double, kKgOff>(data) * mp::V,
      wpi::UnpackStruct<double, kKvOff>(data) *
          frc::ElevatorFeedforward::kv_unit,
      wpi::UnpackStruct<double, kKaOff>(data) *
          frc::ElevatorFeedforward::ka_unit,
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const frc::ElevatorFeedforward& value) {
  wpi::PackStruct<kKsOff>(data, mp::value(value.GetKs()));
  wpi::PackStruct<kKgOff>(data, mp::value(value.GetKg()));
  wpi::PackStruct<kKvOff>(data, mp::value(value.GetKv()));
  wpi::PackStruct<kKaOff>(data, mp::value(value.GetKa()));
}
