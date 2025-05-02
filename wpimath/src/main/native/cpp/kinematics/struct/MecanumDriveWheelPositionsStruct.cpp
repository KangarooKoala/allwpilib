// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/kinematics/struct/MecanumDriveWheelPositionsStruct.h"

namespace {
constexpr size_t kFrontLeftOff = 0;
constexpr size_t kFrontRightOff = kFrontLeftOff + 8;
constexpr size_t kRearLeftOff = kFrontRightOff + 8;
constexpr size_t kRearRightOff = kRearLeftOff + 8;
}  // namespace

using StructType = wpi::Struct<frc::MecanumDriveWheelPositions>;

frc::MecanumDriveWheelPositions StructType::Unpack(
    std::span<const uint8_t> data) {
  return frc::MecanumDriveWheelPositions{
      wpi::UnpackStruct<double, kFrontLeftOff>(data) * mp::m,
      wpi::UnpackStruct<double, kFrontRightOff>(data) * mp::m,
      wpi::UnpackStruct<double, kRearLeftOff>(data) * mp::m,
      wpi::UnpackStruct<double, kRearRightOff>(data) * mp::m,
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const frc::MecanumDriveWheelPositions& value) {
  wpi::PackStruct<kFrontLeftOff>(data, mp::value(value.frontLeft));
  wpi::PackStruct<kFrontRightOff>(data, mp::value(value.frontRight));
  wpi::PackStruct<kRearLeftOff>(data, mp::value(value.rearLeft));
  wpi::PackStruct<kRearRightOff>(data, mp::value(value.rearRight));
}
