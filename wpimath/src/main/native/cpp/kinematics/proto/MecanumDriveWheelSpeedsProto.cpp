// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/kinematics/proto/MecanumDriveWheelSpeedsProto.h"

#include "wpimath/protobuf/kinematics.npb.h"

std::optional<frc::MecanumDriveWheelSpeeds>
wpi::Protobuf<frc::MecanumDriveWheelSpeeds>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufMecanumDriveWheelSpeeds msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return frc::MecanumDriveWheelSpeeds{
      msg.front_left * mp::m / mp::s,
      msg.front_right * mp::m / mp::s,
      msg.rear_left * mp::m / mp::s,
      msg.rear_right * mp::m / mp::s,
  };
}

bool wpi::Protobuf<frc::MecanumDriveWheelSpeeds>::Pack(
    OutputStream& stream, const frc::MecanumDriveWheelSpeeds& value) {
  wpi_proto_ProtobufMecanumDriveWheelSpeeds msg{
      .front_left = mp::value(value.frontLeft),
      .front_right = mp::value(value.frontRight),
      .rear_left = mp::value(value.rearLeft),
      .rear_right = mp::value(value.rearRight),
  };
  return stream.Encode(msg);
}
