// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>
#include <wpi/SmallVector.h>

#include "frc/geometry/Translation2d.h"

using namespace frc;

namespace {

const Translation2d kExpectedData = Translation2d{3.504 * mp::m, 22.9 * mp::m};
}  // namespace

TEST(Translation2dProtoTest, Roundtrip) {
  wpi::ProtobufMessage<decltype(kExpectedData)> message;
  wpi::SmallVector<uint8_t, 64> buf;

  ASSERT_TRUE(message.Pack(buf, kExpectedData));
  auto unpacked_data = message.Unpack(buf);
  ASSERT_TRUE(unpacked_data.has_value());
  EXPECT_EQ(mp::value(kExpectedData.X()), mp::value(unpacked_data->X()));
  EXPECT_EQ(mp::value(kExpectedData.Y()), mp::value(unpacked_data->Y()));
}
