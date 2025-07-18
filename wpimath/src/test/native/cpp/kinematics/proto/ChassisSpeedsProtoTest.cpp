// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>
#include <wpi/SmallVector.h>

#include "frc/kinematics/ChassisSpeeds.h"

using namespace frc;

namespace {

const ChassisSpeeds kExpectedData = ChassisSpeeds{
    2.29 * mp::m / mp::s, 2.2 * mp::m / mp::s, 0.3504 * mp::rad / mp::s};
}  // namespace

TEST(ChassisSpeedsProtoTest, Roundtrip) {
  wpi::ProtobufMessage<decltype(kExpectedData)> message;
  wpi::SmallVector<uint8_t, 64> buf;

  ASSERT_TRUE(message.Pack(buf, kExpectedData));
  auto unpacked_data = message.Unpack(buf);
  ASSERT_TRUE(unpacked_data.has_value());

  EXPECT_EQ(mp::value(kExpectedData.vx), mp::value(unpacked_data->vx));
  EXPECT_EQ(mp::value(kExpectedData.vy), mp::value(unpacked_data->vy));
  EXPECT_EQ(mp::value(kExpectedData.omega), mp::value(unpacked_data->omega));
}
