// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/kinematics/proto/ChassisSpeedsProto.h"

#include "wpimath/protobuf/kinematics.npb.h"

std::optional<frc::ChassisSpeeds> wpi::Protobuf<frc::ChassisSpeeds>::Unpack(
    InputStream& stream) {
  wpi_proto_ProtobufChassisSpeeds msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return frc::ChassisSpeeds{
      msg.vx * mp::m / mp::s,
      msg.vy * mp::m / mp::s,
      msg.omega * mp::rad / mp::s,
  };
}

bool wpi::Protobuf<frc::ChassisSpeeds>::Pack(OutputStream& stream,
                                             const frc::ChassisSpeeds& value) {
  wpi_proto_ProtobufChassisSpeeds msg{
      .vx = mp::value(value.vx),
      .vy = mp::value(value.vy),
      .omega = mp::value(value.omega),
  };
  return stream.Encode(msg);
}
