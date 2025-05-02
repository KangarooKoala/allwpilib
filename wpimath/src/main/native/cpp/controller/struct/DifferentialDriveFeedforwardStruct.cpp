// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/controller/struct/DifferentialDriveFeedforwardStruct.h"

namespace {
constexpr size_t kKvLinearOff = 0;
constexpr size_t kKaLinearOff = kKvLinearOff + 8;
constexpr size_t kKvAngularOff = kKaLinearOff + 8;
constexpr size_t kKaAngularOff = kKvAngularOff + 8;
}  // namespace

frc::DifferentialDriveFeedforward wpi::Struct<
    frc::DifferentialDriveFeedforward>::Unpack(std::span<const uint8_t> data) {
  return {
      wpi::UnpackStruct<double, kKvLinearOff>(data) * mp::V / (mp::m / mp::s),
      wpi::UnpackStruct<double, kKaLinearOff>(data) * mp::V / (mp::m / mp::s2),
      wpi::UnpackStruct<double, kKvAngularOff>(data) * mp::V / (mp::m / mp::s),
      wpi::UnpackStruct<double, kKaAngularOff>(data) * mp::V /
          (mp::m / mp::s2)};
}

void wpi::Struct<frc::DifferentialDriveFeedforward>::Pack(
    std::span<uint8_t> data, const frc::DifferentialDriveFeedforward& value) {
  wpi::PackStruct<kKvLinearOff>(data, mp::value(value.m_kVLinear));
  wpi::PackStruct<kKaLinearOff>(data, mp::value(value.m_kALinear));
  wpi::PackStruct<kKvAngularOff>(data, mp::value(value.m_kVAngular));
  wpi::PackStruct<kKaAngularOff>(data, mp::value(value.m_kAAngular));
}
