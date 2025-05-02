// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/proto/Twist2dProto.h"

#include "wpimath/protobuf/geometry2d.npb.h"

std::optional<frc::Twist2d> wpi::Protobuf<frc::Twist2d>::Unpack(
    InputStream& stream) {
  wpi_proto_ProtobufTwist2d msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return frc::Twist2d{
      msg.dx * mp::m,
      msg.dy * mp::m,
      msg.dtheta * mp::rad,
  };
}

bool wpi::Protobuf<frc::Twist2d>::Pack(OutputStream& stream,
                                       const frc::Twist2d& value) {
  wpi_proto_ProtobufTwist2d msg{
      .dx = mp::value(value.dx),
      .dy = mp::value(value.dy),
      .dtheta = mp::value(value.dtheta),
  };
  return stream.Encode(msg);
}
