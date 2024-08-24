// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"

namespace units {

UNIT_ALIAS_METRIC(frequency, hertz, Hz, mp_units::si::hertz)

UNIT_ADD_CATEGORY_CONCEPT(frequency, mp_units::isq::frequency)

using namespace frequency;

}  // namespace units
