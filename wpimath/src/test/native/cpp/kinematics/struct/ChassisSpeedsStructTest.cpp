// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/kinematics/ChassisSpeeds.h"

using namespace frc;

namespace {

using StructType = wpi::Struct<frc::ChassisSpeeds>;
const ChassisSpeeds kExpectedData{ChassisSpeeds{
    2.29 * mp::m / mp::s, 2.2 * mp::m / mp::s, 0.3504 * mp::rad / mp::s}};
}  // namespace

TEST(ChassisSpeedsStructTest, Roundtrip) {
  uint8_t buffer[StructType::GetSize()];
  std::memset(buffer, 0, StructType::GetSize());
  StructType::Pack(buffer, kExpectedData);

  ChassisSpeeds unpacked_data = StructType::Unpack(buffer);

  EXPECT_EQ(mp::value(kExpectedData.vx), mp::value(unpacked_data.vx));
  EXPECT_EQ(mp::value(kExpectedData.vy), mp::value(unpacked_data.vy));
  EXPECT_EQ(mp::value(kExpectedData.omega), mp::value(unpacked_data.omega));
}
