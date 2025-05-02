// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/proto/Translation2dProto.h"

#include "wpimath/protobuf/geometry2d.npb.h"

std::optional<frc::Translation2d> wpi::Protobuf<frc::Translation2d>::Unpack(
    InputStream& stream) {
  wpi_proto_ProtobufTranslation2d msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return frc::Translation2d{
      msg.x * mp::m,
      msg.y * mp::m,
  };
}

bool wpi::Protobuf<frc::Translation2d>::Pack(OutputStream& stream,
                                             const frc::Translation2d& value) {
  wpi_proto_ProtobufTranslation2d msg{
      .x = mp::value(value.X()),
      .y = mp::value(value.Y()),
  };
  return stream.Encode(msg);
}
