// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/kinematics/MecanumDriveWheelSpeeds.h"

using namespace frc;

namespace {

using StructType = wpi::Struct<frc::MecanumDriveWheelSpeeds>;
const MecanumDriveWheelSpeeds kExpectedData{
    MecanumDriveWheelSpeeds{2.29 * mp::m / mp::s, 17.4 * mp::m / mp::s,
                            4.4 * mp::m / mp::s, 0.229 * mp::m / mp::s}};
}  // namespace

TEST(MecanumDriveWheelSpeedsStructTest, Roundtrip) {
  uint8_t buffer[StructType::GetSize()];
  std::memset(buffer, 0, StructType::GetSize());
  StructType::Pack(buffer, kExpectedData);

  MecanumDriveWheelSpeeds unpacked_data = StructType::Unpack(buffer);

  EXPECT_EQ(mp::value(kExpectedData.frontLeft),
            mp::value(unpacked_data.frontLeft));
  EXPECT_EQ(mp::value(kExpectedData.frontRight),
            mp::value(unpacked_data.frontRight));
  EXPECT_EQ(mp::value(kExpectedData.rearLeft),
            mp::value(unpacked_data.rearLeft));
  EXPECT_EQ(mp::value(kExpectedData.rearRight),
            mp::value(unpacked_data.rearRight));
}
