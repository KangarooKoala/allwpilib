// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/math/kinematics/struct/SwerveModuleStateStruct.hpp"

namespace {
constexpr size_t SPEED_OFF = 0;
constexpr size_t ANGLE_OFF = SPEED_OFF + 8;
}  // namespace

using StructType = wpi::util::Struct<wpi::math::SwerveModuleState>;

wpi::math::SwerveModuleState StructType::Unpack(std::span<const uint8_t> data) {
  return wpi::math::SwerveModuleState{
      wpi::units::meters_per_second_t{
          wpi::util::UnpackStruct<double, SPEED_OFF>(data)},
      wpi::util::UnpackStruct<wpi::math::Rotation2d, ANGLE_OFF>(data),
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const wpi::math::SwerveModuleState& value) {
  wpi::util::PackStruct<SPEED_OFF>(data, value.speed.value());
  wpi::util::PackStruct<ANGLE_OFF>(data, value.angle);
}
