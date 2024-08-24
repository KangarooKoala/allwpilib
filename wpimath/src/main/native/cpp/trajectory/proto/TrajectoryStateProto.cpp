// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/trajectory/proto/TrajectoryStateProto.h"

#include <wpi/ProtoHelper.h>

#include "trajectory.pb.h"

google::protobuf::Message* wpi::Protobuf<frc::Trajectory::State>::New(
    google::protobuf::Arena* arena) {
  return wpi::CreateMessage<wpi::proto::ProtobufTrajectoryState>(arena);
}

frc::Trajectory::State wpi::Protobuf<frc::Trajectory::State>::Unpack(
    const google::protobuf::Message& msg) {
  auto m = static_cast<const wpi::proto::ProtobufTrajectoryState*>(&msg);
  return frc::Trajectory::State{
      m->time() * units::second,
      m->velocity() * units::meters_per_second,
      m->acceleration() * units::meters_per_second_squared,
      wpi::UnpackProtobuf<frc::Pose2d>(m->wpi_pose()),
      m->curvature() * 1_rad / 1_m,
  };
}

void wpi::Protobuf<frc::Trajectory::State>::Pack(
    google::protobuf::Message* msg, const frc::Trajectory::State& value) {
  auto m = static_cast<wpi::proto::ProtobufTrajectoryState*>(msg);
  m->set_time(value.t.value());
  m->set_velocity(value.velocity.value());
  m->set_acceleration(value.acceleration.value());
  wpi::PackProtobuf(m->mutable_pose(), value.pose);
  m->set_curvature(value.curvature.value());
}
