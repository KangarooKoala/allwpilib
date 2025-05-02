// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/controller/proto/DifferentialDriveWheelVoltagesProto.h"

#include <optional>

#include "wpimath/protobuf/controller.npb.h"

std::optional<frc::DifferentialDriveWheelVoltages> wpi::Protobuf<
    frc::DifferentialDriveWheelVoltages>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufDifferentialDriveWheelVoltages msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return frc::DifferentialDriveWheelVoltages{
      msg.left * mp::V,
      msg.right * mp::V,
  };
}

bool wpi::Protobuf<frc::DifferentialDriveWheelVoltages>::Pack(
    OutputStream& stream, const frc::DifferentialDriveWheelVoltages& value) {
  wpi_proto_ProtobufDifferentialDriveWheelVoltages msg{
      .left = mp::value(value.left),
      .right = mp::value(value.right),
  };
  return stream.Encode(msg);
}
