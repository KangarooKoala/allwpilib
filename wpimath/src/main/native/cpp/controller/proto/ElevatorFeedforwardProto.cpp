// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/controller/proto/ElevatorFeedforwardProto.h"

#include <wpi/ProtoHelper.h>

#include "controller.pb.h"

google::protobuf::Message* wpi::Protobuf<frc::ElevatorFeedforward>::New(
    google::protobuf::Arena* arena) {
  return wpi::CreateMessage<wpi::proto::ProtobufElevatorFeedforward>(arena);
}

frc::ElevatorFeedforward wpi::Protobuf<frc::ElevatorFeedforward>::Unpack(
    const google::protobuf::Message& msg) {
  auto m = static_cast<const wpi::proto::ProtobufElevatorFeedforward*>(&msg);
  return frc::ElevatorFeedforward{
      m->ks() * units::volt,
      m->kg() * units::volt,
      m->kv() * frc::ElevatorFeedforward::kv_unit,
      m->ka() * frc::ElevatorFeedforward::ka_unit,
  };
}

void wpi::Protobuf<frc::ElevatorFeedforward>::Pack(
    google::protobuf::Message* msg, const frc::ElevatorFeedforward& value) {
  auto m = static_cast<wpi::proto::ProtobufElevatorFeedforward*>(msg);
  m->set_ks(value.GetKs().value());
  m->set_kg(value.GetKg().value());
  m->set_kv(value.GetKv().value());
  m->set_ka(value.GetKa().value());
}
