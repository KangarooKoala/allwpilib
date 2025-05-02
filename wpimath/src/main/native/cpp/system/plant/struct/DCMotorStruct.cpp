// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/system/plant/struct/DCMotorStruct.h"

namespace {
constexpr size_t kNominalVoltageOff = 0;
constexpr size_t kStallTorqueOff = kNominalVoltageOff + 8;
constexpr size_t kStallCurrentOff = kStallTorqueOff + 8;
constexpr size_t kFreeCurrentOff = kStallCurrentOff + 8;
constexpr size_t kFreeSpeedOff = kFreeCurrentOff + 8;
}  // namespace

using StructType = wpi::Struct<frc::DCMotor>;

frc::DCMotor StructType::Unpack(std::span<const uint8_t> data) {
  return frc::DCMotor{
      wpi::UnpackStruct<double, kNominalVoltageOff>(data) * mp::V,
      wpi::UnpackStruct<double, kStallTorqueOff>(data) * mp::N * mp::m,
      wpi::UnpackStruct<double, kStallCurrentOff>(data) * mp::A,
      wpi::UnpackStruct<double, kFreeCurrentOff>(data) * mp::A,
      wpi::UnpackStruct<double, kFreeSpeedOff>(data) * mp::rad / mp::s,
  };
}

void StructType::Pack(std::span<uint8_t> data, const frc::DCMotor& value) {
  wpi::PackStruct<kNominalVoltageOff>(data, mp::value(value.nominalVoltage));
  wpi::PackStruct<kStallTorqueOff>(data, mp::value(value.stallTorque));
  wpi::PackStruct<kStallCurrentOff>(data, mp::value(value.stallCurrent));
  wpi::PackStruct<kFreeCurrentOff>(data, mp::value(value.freeCurrent));
  wpi::PackStruct<kFreeSpeedOff>(data, mp::value(value.freeSpeed));
}
