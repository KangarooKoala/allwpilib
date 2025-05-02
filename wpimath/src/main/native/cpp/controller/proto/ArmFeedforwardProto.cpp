// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/controller/proto/ArmFeedforwardProto.h"

#include <optional>

#include "wpimath/protobuf/controller.npb.h"

std::optional<frc::ArmFeedforward> wpi::Protobuf<frc::ArmFeedforward>::Unpack(
    InputStream& stream) {
  wpi_proto_ProtobufArmFeedforward msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return frc::ArmFeedforward{
      msg.ks * mp::V,
      msg.kg * mp::V,
      msg.kv * frc::ArmFeedforward::kv_unit,
      msg.ka * frc::ArmFeedforward::ka_unit,
  };
}

bool wpi::Protobuf<frc::ArmFeedforward>::Pack(
    OutputStream& stream, const frc::ArmFeedforward& value) {
  wpi_proto_ProtobufArmFeedforward msg{
      .ks = mp::value(value.GetKs()),
      .kg = mp::value(value.GetKg()),
      .kv = mp::value(value.GetKv()),
      .ka = mp::value(value.GetKa()),
      .dt = 0,
  };
  return stream.Encode(msg);
}
