// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/protobuf/Protobuf.h>

#include "frc/controller/SimpleMotorFeedforward.h"
#include "frc/units.h"
#include "pb.h"
#include "wpimath/protobuf/controller.npb.h"

// Everything is converted into units for
// frc::SimpleMotorFeedforward<mp::m> or
// frc::SimpleMotorFeedforward<mp::rad>

template <mp::Unit auto Distance>
  requires mp::UnitOf<decltype(Distance), mp::length> ||
           mp::UnitOf<decltype(Distance), mp::angle>
struct wpi::Protobuf<frc::SimpleMotorFeedforward<Distance>> {
  // Because all instantiations of
  // wpi::Protobuf<frc::SimpleMotorFeedforward<Distance>> use the same
  // MessageStruct type, it doesn't matter if we use Distance or
  // mp::get_base_unit(Distance) in InputStream and OutputStream.
  using MessageStruct = wpi_proto_ProtobufSimpleMotorFeedforward;
  using InputStream =
      wpi::ProtoInputStream<frc::SimpleMotorFeedforward<Distance>>;
  using OutputStream =
      wpi::ProtoOutputStream<frc::SimpleMotorFeedforward<Distance>>;

  static std::optional<frc::SimpleMotorFeedforward<Distance>> Unpack(
      InputStream& stream) {
    using BaseFeedforward =
        frc::SimpleMotorFeedforward<mp::get_base_unit(Distance)>;
    wpi_proto_ProtobufSimpleMotorFeedforward msg;
    if (!stream.Decode(msg)) {
      return {};
    }

    return frc::SimpleMotorFeedforward<Distance>{
        msg.ks * mp::V,
        msg.kv * BaseFeedforward::kv_unit,
        msg.ka * BaseFeedforward::ka_unit,
        msg.dt * mp::s,
    };
  }

  static bool Pack(OutputStream& stream,
                   const frc::SimpleMotorFeedforward<Distance>& value) {
    using BaseFeedforward =
        frc::SimpleMotorFeedforward<mp::get_base_unit(Distance)>;
    wpi_proto_ProtobufSimpleMotorFeedforward msg{
        .ks = mp::value(value.GetKs().in(mp::V)),
        .kv = mp::value(value.GetKv().in(BaseFeedforward::kv_unit)),
        .ka = mp::value(value.GetKa().in(BaseFeedforward::ka_unit)),
        .dt = mp::value(value.GetDt().in(mp::s)),
    };
    return stream.Encode(msg);
  }
};
