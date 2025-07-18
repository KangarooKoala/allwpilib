// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>
#include <wpi/SmallVector.h>

#include "frc/geometry/Ellipse2d.h"

using namespace frc;

namespace {

const Ellipse2d kExpectedData{Pose2d{Translation2d{0.191 * mp::m, 2.2 * mp::m},
                                     Rotation2d{22.9 * mp::rad}},
                              1.2 * mp::m, 2.3 * mp::m};
}  // namespace

TEST(Ellipse2dProtoTest, Roundtrip) {
  wpi::ProtobufMessage<decltype(kExpectedData)> message;
  wpi::SmallVector<uint8_t, 64> buf;

  ASSERT_TRUE(message.Pack(buf, kExpectedData));
  auto unpacked_data = message.Unpack(buf);
  ASSERT_TRUE(unpacked_data.has_value());

  EXPECT_EQ(kExpectedData.Center(), unpacked_data->Center());
  EXPECT_EQ(kExpectedData.XSemiAxis(), unpacked_data->XSemiAxis());
  EXPECT_EQ(kExpectedData.YSemiAxis(), unpacked_data->YSemiAxis());
}
