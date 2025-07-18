// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/kinematics/DifferentialDriveWheelPositions.h"

using namespace frc;

namespace {

using StructType = wpi::Struct<frc::DifferentialDriveWheelPositions>;
const DifferentialDriveWheelPositions kExpectedData{
    DifferentialDriveWheelPositions{1.74 * mp::m, 35.04 * mp::m}};
}  // namespace

TEST(DifferentialDriveWheelPositionsStructTest, Roundtrip) {
  uint8_t buffer[StructType::GetSize()];
  std::memset(buffer, 0, StructType::GetSize());
  StructType::Pack(buffer, kExpectedData);

  DifferentialDriveWheelPositions unpacked_data = StructType::Unpack(buffer);

  EXPECT_EQ(mp::value(kExpectedData.left), mp::value(unpacked_data.left));
  EXPECT_EQ(mp::value(kExpectedData.right), mp::value(unpacked_data.right));
}
