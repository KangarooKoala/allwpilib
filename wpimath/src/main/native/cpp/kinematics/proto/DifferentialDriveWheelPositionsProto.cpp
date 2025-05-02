// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/kinematics/proto/DifferentialDriveWheelPositionsProto.h"

#include "wpimath/protobuf/kinematics.npb.h"

std::optional<frc::DifferentialDriveWheelPositions> wpi::Protobuf<
    frc::DifferentialDriveWheelPositions>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufDifferentialDriveWheelPositions msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return frc::DifferentialDriveWheelPositions{
      msg.left * mp::m,
      msg.right * mp::m,
  };
}

bool wpi::Protobuf<frc::DifferentialDriveWheelPositions>::Pack(
    OutputStream& stream, const frc::DifferentialDriveWheelPositions& value) {
  wpi_proto_ProtobufDifferentialDriveWheelPositions msg{
      .left = mp::value(value.left),
      .right = mp::value(value.right),
  };
  return stream.Encode(msg);
}
