// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units2/base.h"

namespace units {

UNIT_ALIAS_METRIC(voltage, volt, V, mp_units::si::volt)
UNIT_DEFINE(voltage, statvolt, statV,
            mp_units::mag_ratio<1'000'000, 299'792'458>* volt)
UNIT_DEFINE(voltage, abvolt, abV, mp_units::mag_power<10, -8>* volt)

UNIT_ADD_CATEGORY_CONCEPT(voltage, mp_units::isq::voltage)

using namespace voltage;

}  // namespace units
