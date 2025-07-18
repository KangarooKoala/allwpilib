// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>
#include <wpi/SmallVector.h>

#include "frc/trajectory/Trajectory.h"

using namespace frc;

namespace {

using ProtoType = wpi::Protobuf<frc::Trajectory::State>;

const Trajectory::State kExpectedData =
    Trajectory::State{1.91 * mp::s, 4.4 * mp::m / mp::s, 17.4 * mp::m / mp::s2,
                      Pose2d{Translation2d{1.74 * mp::m, 19.1 * mp::m},
                             Rotation2d{22.9 * mp::rad}},
                      0.174 * mp::rad / mp::m};
}  // namespace

TEST(TrajectoryStateProtoTest, Roundtrip) {
  wpi::ProtobufMessage<decltype(kExpectedData)> message;
  wpi::SmallVector<uint8_t, 64> buf;

  ASSERT_TRUE(message.Pack(buf, kExpectedData));
  auto unpacked_data = message.Unpack(buf);
  ASSERT_TRUE(unpacked_data.has_value());

  EXPECT_EQ(mp::value(kExpectedData.t), mp::value(unpacked_data->t));
  EXPECT_EQ(mp::value(kExpectedData.velocity),
            mp::value(unpacked_data->velocity));
  EXPECT_EQ(mp::value(kExpectedData.acceleration),
            mp::value(unpacked_data->acceleration));
  EXPECT_EQ(kExpectedData.pose, unpacked_data->pose);
  EXPECT_EQ(mp::value(kExpectedData.curvature),
            mp::value(unpacked_data->curvature));
}
