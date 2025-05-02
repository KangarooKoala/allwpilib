// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/kinematics/proto/MecanumDriveWheelPositionsProto.h"

#include "wpimath/protobuf/kinematics.npb.h"

std::optional<frc::MecanumDriveWheelPositions>
wpi::Protobuf<frc::MecanumDriveWheelPositions>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufMecanumDriveWheelPositions msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return frc::MecanumDriveWheelPositions{
      msg.front_left * mp::m,
      msg.front_right * mp::m,
      msg.rear_left * mp::m,
      msg.rear_right * mp::m,
  };
}

bool wpi::Protobuf<frc::MecanumDriveWheelPositions>::Pack(
    OutputStream& stream, const frc::MecanumDriveWheelPositions& value) {
  wpi_proto_ProtobufMecanumDriveWheelPositions msg{
      .front_left = mp::value(value.frontLeft),
      .front_right = mp::value(value.frontRight),
      .rear_left = mp::value(value.rearLeft),
      .rear_right = mp::value(value.rearRight),
  };
  return stream.Encode(msg);
}
