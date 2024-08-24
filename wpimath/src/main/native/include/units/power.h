// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"

namespace units {

UNIT_ALIAS_METRIC(power, watt, W, mp_units::si::watt)
UNIT_DEFINE(power, horsepower, hp, mp_units::mag_ratio<7457, 10>* watt)

UNIT_ADD_CATEGORY_CONCEPT(power, mp_units::isq::power)

using namespace power;

}  // namespace units
