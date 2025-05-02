// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/proto/Rotation2dProto.h"

#include "wpimath/protobuf/geometry2d.npb.h"

std::optional<frc::Rotation2d> wpi::Protobuf<frc::Rotation2d>::Unpack(
    InputStream& stream) {
  wpi_proto_ProtobufRotation2d msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return frc::Rotation2d{
      msg.value * mp::rad,
  };
}

bool wpi::Protobuf<frc::Rotation2d>::Pack(OutputStream& stream,
                                          const frc::Rotation2d& value) {
  wpi_proto_ProtobufRotation2d msg{
      .value = mp::value(value.Radians()),
  };
  return stream.Encode(msg);
}
