// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/controller/proto/ElevatorFeedforwardProto.h"

#include <optional>

#include "wpimath/protobuf/controller.npb.h"

std::optional<frc::ElevatorFeedforward>
wpi::Protobuf<frc::ElevatorFeedforward>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufElevatorFeedforward msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return frc::ElevatorFeedforward{
      msg.ks * mp::V,
      msg.kg * mp::V,
      msg.kv * frc::ElevatorFeedforward::kv_unit,
      msg.ka * frc::ElevatorFeedforward::ka_unit,
  };
}

bool wpi::Protobuf<frc::ElevatorFeedforward>::Pack(
    OutputStream& stream, const frc::ElevatorFeedforward& value) {
  wpi_proto_ProtobufElevatorFeedforward msg{
      .ks = mp::value(value.GetKs()),
      .kg = mp::value(value.GetKg()),
      .kv = mp::value(value.GetKv()),
      .ka = mp::value(value.GetKa()),
      .dt = 0,
  };
  return stream.Encode(msg);
}
