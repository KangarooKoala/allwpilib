// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/system/plant/proto/DCMotorProto.h"

#include <optional>

#include "wpimath/protobuf/plant.npb.h"

std::optional<frc::DCMotor> wpi::Protobuf<frc::DCMotor>::Unpack(
    InputStream& stream) {
  wpi_proto_ProtobufDCMotor msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return frc::DCMotor{
      msg.nominal_voltage * mp::V,      msg.stall_torque * mp::N * mp::m,
      msg.stall_current * mp::A,        msg.free_current * mp::A,
      msg.free_speed * mp::rad / mp::s,
  };
}

bool wpi::Protobuf<frc::DCMotor>::Pack(OutputStream& stream,
                                       const frc::DCMotor& value) {
  wpi_proto_ProtobufDCMotor msg{
      .nominal_voltage = mp::value(value.nominalVoltage),
      .stall_torque = mp::value(value.stallTorque),
      .stall_current = mp::value(value.stallCurrent),
      .free_current = mp::value(value.freeCurrent),
      .free_speed = mp::value(value.freeSpeed),
  };
  return stream.Encode(msg);
}
