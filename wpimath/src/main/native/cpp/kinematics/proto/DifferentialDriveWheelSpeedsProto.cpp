// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/kinematics/proto/DifferentialDriveWheelSpeedsProto.h"

#include "wpimath/protobuf/kinematics.npb.h"

std::optional<frc::DifferentialDriveWheelSpeeds>
wpi::Protobuf<frc::DifferentialDriveWheelSpeeds>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufDifferentialDriveWheelSpeeds msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return frc::DifferentialDriveWheelSpeeds{
      msg.left * mp::m / mp::s,
      msg.right * mp::m / mp::s,
  };
}

bool wpi::Protobuf<frc::DifferentialDriveWheelSpeeds>::Pack(
    OutputStream& stream, const frc::DifferentialDriveWheelSpeeds& value) {
  wpi_proto_ProtobufDifferentialDriveWheelSpeeds msg{
      .left = mp::value(value.left),
      .right = mp::value(value.right),
  };
  return stream.Encode(msg);
}
