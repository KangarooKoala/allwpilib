// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/geometry/Twist2d.h"

using namespace frc;

namespace {

using StructType = wpi::Struct<frc::Twist2d>;
const Twist2d kExpectedData{
    Twist2d{2.29 * mp::m, 35.04 * mp::m, 35.04 * mp::rad}};
}  // namespace

TEST(Twist2dStructTest, Roundtrip) {
  uint8_t buffer[StructType::GetSize()];
  std::memset(buffer, 0, StructType::GetSize());
  StructType::Pack(buffer, kExpectedData);

  Twist2d unpacked_data = StructType::Unpack(buffer);

  EXPECT_EQ(mp::value(kExpectedData.dx), mp::value(unpacked_data.dx));
  EXPECT_EQ(mp::value(kExpectedData.dy), mp::value(unpacked_data.dy));
  EXPECT_EQ(mp::value(kExpectedData.dtheta), mp::value(unpacked_data.dtheta));
}
