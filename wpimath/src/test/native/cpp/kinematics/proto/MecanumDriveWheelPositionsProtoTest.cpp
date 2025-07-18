// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>
#include <wpi/SmallVector.h>

#include "frc/kinematics/MecanumDriveWheelPositions.h"

using namespace frc;

namespace {

const MecanumDriveWheelPositions kExpectedData = MecanumDriveWheelPositions{
    17.4 * mp::m, 2.29 * mp::m, 22.9 * mp::m, 1.74 * mp::m};
}  // namespace

TEST(MecanumDriveWheelPositionsProtoTest, Roundtrip) {
  wpi::ProtobufMessage<decltype(kExpectedData)> message;
  wpi::SmallVector<uint8_t, 64> buf;

  ASSERT_TRUE(message.Pack(buf, kExpectedData));
  auto unpacked_data = message.Unpack(buf);
  ASSERT_TRUE(unpacked_data.has_value());

  EXPECT_EQ(mp::value(kExpectedData.frontLeft),
            mp::value(unpacked_data->frontLeft));
  EXPECT_EQ(mp::value(kExpectedData.frontRight),
            mp::value(unpacked_data->frontRight));
  EXPECT_EQ(mp::value(kExpectedData.rearLeft),
            mp::value(unpacked_data->rearLeft));
  EXPECT_EQ(mp::value(kExpectedData.rearRight),
            mp::value(unpacked_data->rearRight));
}
