// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/proto/Ellipse2dProto.h"

#include <wpi/protobuf/ProtobufCallbacks.h>

#include "wpimath/protobuf/geometry2d.npb.h"

std::optional<frc::Ellipse2d> wpi::Protobuf<frc::Ellipse2d>::Unpack(
    InputStream& stream) {
  wpi::UnpackCallback<frc::Pose2d> pose;
  wpi_proto_ProtobufEllipse2d msg{
      .center = pose.Callback(),
      .xSemiAxis = 0,
      .ySemiAxis = 0,
  };
  if (!stream.Decode(msg)) {
    return {};
  }

  auto ipose = pose.Items();

  if (ipose.empty()) {
    return {};
  }

  return frc::Ellipse2d{
      ipose[0],
      msg.xSemiAxis * mp::m,
      msg.ySemiAxis * mp::m,
  };
}

bool wpi::Protobuf<frc::Ellipse2d>::Pack(OutputStream& stream,
                                         const frc::Ellipse2d& value) {
  wpi::PackCallback pose{&value.Center()};
  wpi_proto_ProtobufEllipse2d msg{
      .center = pose.Callback(),
      .xSemiAxis = mp::value(value.XSemiAxis()),
      .ySemiAxis = mp::value(value.YSemiAxis()),
  };
  return stream.Encode(msg);
}
