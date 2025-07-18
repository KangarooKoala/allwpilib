// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>
#include <wpi/SmallVector.h>

#include "frc/controller/ElevatorFeedforward.h"

using namespace frc;

namespace {

static constexpr auto Ks = 1.91 * mp::V;
static constexpr auto Kg = 2.29 * mp::V;
static constexpr auto Kv = 35.04 * mp::V / (mp::m / mp::s);
static constexpr auto Ka = 1.74 * mp::V / (mp::m / mp::s2);

constexpr ElevatorFeedforward kExpectedData{Ks, Kg, Kv, Ka};
}  // namespace

TEST(ElevatorFeedforwardProtoTest, Roundtrip) {
  wpi::ProtobufMessage<decltype(kExpectedData)> message;
  wpi::SmallVector<uint8_t, 64> buf;

  ASSERT_TRUE(message.Pack(buf, kExpectedData));
  auto unpacked_data = message.Unpack(buf);
  ASSERT_TRUE(unpacked_data.has_value());

  EXPECT_EQ(mp::value(kExpectedData.GetKs()),
            mp::value(unpacked_data->GetKs()));
  EXPECT_EQ(mp::value(kExpectedData.GetKg()),
            mp::value(unpacked_data->GetKg()));
  EXPECT_EQ(mp::value(kExpectedData.GetKv()),
            mp::value(unpacked_data->GetKv()));
  EXPECT_EQ(mp::value(kExpectedData.GetKa()),
            mp::value(unpacked_data->GetKa()));
}
