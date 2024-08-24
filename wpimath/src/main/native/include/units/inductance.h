// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"

namespace units {

UNIT_ALIAS_METRIC(inductance, henry, H, mp_units::si::henry)

UNIT_ADD_CATEGORY_CONCEPT(inductance, mp_units::isq::inductance)

using namespace inductance;

}  // namespace units
