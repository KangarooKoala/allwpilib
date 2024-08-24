// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"

namespace units {

UNIT_ALIAS_METRIC(luminous_intensity, candela, cd, mp_units::si::candela)

UNIT_ADD_CATEGORY_CONCEPT(luminous_intensity, mp_units::isq::luminous_intensity)

using namespace luminous_intensity;

}  // namespace units
