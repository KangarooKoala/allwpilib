// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"

namespace units {

UNIT_ALIAS_LARGE_METRIC_AND_BINARY(data, byte, B, mp_units::iec80000::byte)
UNIT_ALIAS_LARGE_METRIC_AND_BINARY(data, bit, b, mp_units::iec80000::bit)

UNIT_ADD_CATEGORY_CONCEPT(data, mp_units::iec80000::storage_capacity)

using namespace data;

}  // namespace units
