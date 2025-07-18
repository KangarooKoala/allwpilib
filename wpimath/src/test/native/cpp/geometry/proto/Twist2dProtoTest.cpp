// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>
#include <wpi/SmallVector.h>

#include "frc/geometry/Twist2d.h"

using namespace frc;

namespace {

const Twist2d kExpectedData =
    Twist2d{2.29 * mp::m, 35.04 * mp::m, 35.04 * mp::rad};
}  // namespace

TEST(Twist2dProtoTest, Roundtrip) {
  wpi::ProtobufMessage<decltype(kExpectedData)> message;
  wpi::SmallVector<uint8_t, 64> buf;

  ASSERT_TRUE(message.Pack(buf, kExpectedData));
  auto unpacked_data = message.Unpack(buf);
  ASSERT_TRUE(unpacked_data.has_value());

  EXPECT_EQ(mp::value(kExpectedData.dx), mp::value(unpacked_data->dx));
  EXPECT_EQ(mp::value(kExpectedData.dy), mp::value(unpacked_data->dy));
  EXPECT_EQ(mp::value(kExpectedData.dtheta), mp::value(unpacked_data->dtheta));
}
