// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/controller/proto/DifferentialDriveFeedforwardProto.h"

#include "wpimath/protobuf/controller.npb.h"

std::optional<frc::DifferentialDriveFeedforward>
wpi::Protobuf<frc::DifferentialDriveFeedforward>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufDifferentialDriveFeedforward msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return frc::DifferentialDriveFeedforward{
      msg.kv_linear * mp::V / (mp::m / mp::s),
      msg.ka_linear * mp::V / (mp::m / mp::s2),
      msg.kv_angular * mp::V / (mp::m / mp::s),
      msg.ka_angular * mp::V / (mp::m / mp::s2),
  };
}

bool wpi::Protobuf<frc::DifferentialDriveFeedforward>::Pack(
    OutputStream& stream, const frc::DifferentialDriveFeedforward& value) {
  wpi_proto_ProtobufDifferentialDriveFeedforward msg{
      .kv_linear = mp::value(value.m_kVLinear),
      .ka_linear = mp::value(value.m_kALinear),
      .kv_angular = mp::value(value.m_kVAngular),
      .ka_angular = mp::value(value.m_kAAngular),
  };
  return stream.Encode(msg);
}
