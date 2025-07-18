// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>
#include <wpi/SmallVector.h>

#include "frc/kinematics/MecanumDriveWheelSpeeds.h"

using namespace frc;

namespace {

const MecanumDriveWheelSpeeds kExpectedData =
    MecanumDriveWheelSpeeds{2.29 * mp::m / mp::s, 17.4 * mp::m / mp::s,
                            4.4 * mp::m / mp::s, 0.229 * mp::m / mp::s};
}  // namespace

TEST(MecanumDriveWheelSpeedsProtoTest, Roundtrip) {
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
