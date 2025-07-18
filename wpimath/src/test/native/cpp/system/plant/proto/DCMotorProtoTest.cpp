// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>
#include <wpi/SmallVector.h>

#include "frc/system/plant/DCMotor.h"

using namespace frc;

using ProtoType = wpi::Protobuf<frc::DCMotor>;

inline constexpr DCMotor kExpectedData =
    DCMotor{1.91 * mp::V, 19.1 * mp::N* mp::m,   1.74 * mp::A,
            2.29 * mp::A, 2.2 * mp::rad / mp::s, 2};

TEST(DCMotorProtoTest, Roundtrip) {
  wpi::ProtobufMessage<decltype(kExpectedData)> message;
  wpi::SmallVector<uint8_t, 64> buf;

  ASSERT_TRUE(message.Pack(buf, kExpectedData));
  auto unpacked_data = message.Unpack(buf);
  ASSERT_TRUE(unpacked_data.has_value());

  EXPECT_EQ(mp::value(kExpectedData.nominalVoltage),
            mp::value(unpacked_data->nominalVoltage));
  EXPECT_EQ(mp::value(kExpectedData.stallTorque),
            mp::value(unpacked_data->stallTorque));
  EXPECT_EQ(mp::value(kExpectedData.stallCurrent),
            mp::value(unpacked_data->stallCurrent));
  EXPECT_EQ(mp::value(kExpectedData.freeCurrent),
            mp::value(unpacked_data->freeCurrent));
  EXPECT_EQ(mp::value(kExpectedData.freeSpeed),
            mp::value(unpacked_data->freeSpeed));
  EXPECT_EQ(mp::value(kExpectedData.R), mp::value(unpacked_data->R));
  EXPECT_EQ(mp::value(kExpectedData.Kv), mp::value(unpacked_data->Kv));
  EXPECT_EQ(mp::value(kExpectedData.Kt), mp::value(unpacked_data->Kt));
}
