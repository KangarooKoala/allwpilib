// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units2/base.h"

namespace units {

UNIT_DEFINE_LARGE_METRIC_AND_BINARY(data_transfer_rate, bytes_per_second, Bps,
                                    mp_units::iec80000::byte /
                                        mp_units::si::second)
UNIT_DEFINE_LARGE_METRIC_AND_BINARY(data_transfer_rate, bits_per_second, bps,
                                    mp_units::iec80000::bit /
                                        mp_units::si::second)

UNIT_ADD_CATEGORY_CONCEPT(data_transfer_rate, mp_units::iec80000::transfer_rate)

using namespace data_transfer_rate;

}  // namespace units
