// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/controller/DifferentialDriveWheelVoltages.h"

using namespace frc;

namespace {

using StructType = wpi::Struct<frc::DifferentialDriveWheelVoltages>;
const DifferentialDriveWheelVoltages kExpectedData{
    DifferentialDriveWheelVoltages{0.174 * mp::V, 0.191 * mp::V}};
}  // namespace

TEST(DifferentialDriveWheelVoltagesStructTest, Roundtrip) {
  uint8_t buffer[StructType::GetSize()];
  std::memset(buffer, 0, StructType::GetSize());
  StructType::Pack(buffer, kExpectedData);

  DifferentialDriveWheelVoltages unpacked_data = StructType::Unpack(buffer);

  EXPECT_EQ(mp::value(kExpectedData.left), mp::value(unpacked_data.left));
  EXPECT_EQ(mp::value(kExpectedData.right), mp::value(unpacked_data.right));
}
