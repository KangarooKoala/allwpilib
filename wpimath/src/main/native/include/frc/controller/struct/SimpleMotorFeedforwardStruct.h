// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/struct/Struct.h>

#include "frc/controller/SimpleMotorFeedforward.h"
#include "frc/units.h"

// Everything is converted into units for
// frc::SimpleMotorFeedforward<mp::m> or
// frc::SimpleMotorFeedforward<mp::rad>

template <mp::Unit auto Distance>
  requires mp::UnitOf<Distance, mp::length> || mp::UnitOf<Distance, mp::angle>
struct wpi::Struct<frc::SimpleMotorFeedforward<Distance>> {
  static constexpr std::string_view GetTypeName() {
    return "SimpleMotorFeedforward";
  }
  static constexpr size_t GetSize() { return 32; }
  static constexpr std::string_view GetSchema() {
    return "double ks;double kv;double ka;double dt";
  }

  static frc::SimpleMotorFeedforward<Distance> Unpack(
      std::span<const uint8_t> data) {
    using BaseFeedforward =
        frc::SimpleMotorFeedforward<mp::get_base_unit(Distance)>;
    constexpr size_t kKsOff = 0;
    constexpr size_t kKvOff = kKsOff + 8;
    constexpr size_t kKaOff = kKvOff + 8;
    constexpr size_t kDtOff = kKaOff + 8;
    return {wpi::UnpackStruct<double, kKsOff>(data) * mp::V,
            wpi::UnpackStruct<double, kKvOff>(data) * BaseFeedforward::kv_unit,
            wpi::UnpackStruct<double, kKaOff>(data) * BaseFeedforward::ka_unit,
            wpi::UnpackStruct<double, kDtOff>(data) * mp::s};
  }

  static void Pack(std::span<uint8_t> data,
                   const frc::SimpleMotorFeedforward<Distance>& value) {
    using BaseFeedforward =
        frc::SimpleMotorFeedforward<mp::get_base_unit(Distance)>;
    constexpr size_t kKsOff = 0;
    constexpr size_t kKvOff = kKsOff + 8;
    constexpr size_t kKaOff = kKvOff + 8;
    constexpr size_t kDtOff = kKaOff + 8;
    wpi::PackStruct<kKsOff>(data, mp::value(value.GetKs().in(mp::V)));
    wpi::PackStruct<kKvOff>(
        data, mp::value(value.GetKv().in(BaseFeedforward::kv_unit)));
    wpi::PackStruct<kKaOff>(
        data, mp::value(value.GetKa().in(BaseFeedforward::ka_unit)));
    wpi::PackStruct<kDtOff>(data, mp::value(value.GetDt().in(mp::s)));
  }
};

static_assert(wpi::StructSerializable<frc::SimpleMotorFeedforward<mp::m>>);
static_assert(wpi::StructSerializable<frc::SimpleMotorFeedforward<mp::cm>>);
static_assert(wpi::StructSerializable<frc::SimpleMotorFeedforward<mp::rad>>);
