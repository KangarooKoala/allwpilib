// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"

namespace units {

UNIT_ALIAS(concentration, ppm, ppm, mp_units::ppm)
UNIT_DEFINE(concentration, ppb, ppb, mp_units::mag_ratio<1, 1000>* ppm)
UNIT_DEFINE(concentration, ppt, ppt, mp_units::mag_ratio<1, 1000>* ppb)
UNIT_ALIAS(concentration, percent, pct, mp_units::percent)

UNIT_ADD_CATEGORY_CONCEPT(concentration, mp_units::dimensionless)

using namespace concentration;

}  // namespace units
