// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/proto/Twist3dProto.h"

#include "wpimath/protobuf/geometry3d.npb.h"

std::optional<frc::Twist3d> wpi::Protobuf<frc::Twist3d>::Unpack(
    InputStream& stream) {
  wpi_proto_ProtobufTwist3d msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return frc::Twist3d{
      msg.dx * mp::m,   msg.dy * mp::m,   msg.dz * mp::m,
      msg.rx * mp::rad, msg.ry * mp::rad, msg.rz * mp::rad,
  };
}

bool wpi::Protobuf<frc::Twist3d>::Pack(OutputStream& stream,
                                       const frc::Twist3d& value) {
  wpi_proto_ProtobufTwist3d msg{
      .dx = mp::value(value.dx),
      .dy = mp::value(value.dy),
      .dz = mp::value(value.dz),
      .rx = mp::value(value.rx),
      .ry = mp::value(value.ry),
      .rz = mp::value(value.rz),
  };
  return stream.Encode(msg);
}
