// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/trajectory/proto/TrajectoryStateProto.h"

#include <utility>

#include <wpi/protobuf/ProtobufCallbacks.h>

#include "wpimath/protobuf/trajectory.npb.h"

std::optional<frc::Trajectory::State>
wpi::Protobuf<frc::Trajectory::State>::Unpack(InputStream& stream) {
  wpi::UnpackCallback<frc::Pose2d> pose;
  wpi_proto_ProtobufTrajectoryState msg;
  msg.pose = pose.Callback();

  if (!stream.Decode(msg)) {
    return {};
  }

  auto ipose = pose.Items();

  if (ipose.empty()) {
    return {};
  }

  return frc::Trajectory::State{
      msg.time * mp::s,
      msg.velocity * mp::m / mp::s,
      msg.acceleration * mp::m / mp::s2,
      std::move(ipose[0]),
      msg.curvature * mp::rad / mp::m,
  };
}

bool wpi::Protobuf<frc::Trajectory::State>::Pack(
    OutputStream& stream, const frc::Trajectory::State& value) {
  wpi::PackCallback pose{&value.pose};
  wpi_proto_ProtobufTrajectoryState msg{
      .time = mp::value(value.t),
      .velocity = mp::value(value.velocity),
      .acceleration = mp::value(value.acceleration),
      .pose = pose.Callback(),
      .curvature = mp::value(value.curvature),
  };
  return stream.Encode(msg);
}
