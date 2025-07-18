// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/geometry/Twist3d.h"

using namespace frc;

namespace {

using StructType = wpi::Struct<frc::Twist3d>;
const Twist3d kExpectedData{Twist3d{1.1 * mp::m, 2.29 * mp::m, 35.04 * mp::m,
                                    0.174 * mp::rad, 19.1 * mp::rad,
                                    4.4 * mp::rad}};
}  // namespace

TEST(Twist3dStructTest, Roundtrip) {
  uint8_t buffer[StructType::GetSize()];
  std::memset(buffer, 0, StructType::GetSize());
  StructType::Pack(buffer, kExpectedData);

  Twist3d unpacked_data = StructType::Unpack(buffer);

  EXPECT_EQ(mp::value(kExpectedData.dx), mp::value(unpacked_data.dx));
  EXPECT_EQ(mp::value(kExpectedData.dy), mp::value(unpacked_data.dy));
  EXPECT_EQ(mp::value(kExpectedData.dz), mp::value(unpacked_data.dz));
  EXPECT_EQ(mp::value(kExpectedData.rx), mp::value(unpacked_data.rx));
  EXPECT_EQ(mp::value(kExpectedData.ry), mp::value(unpacked_data.ry));
  EXPECT_EQ(mp::value(kExpectedData.rz), mp::value(unpacked_data.rz));
}
