// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"

namespace units {

UNIT_ALIAS(capacitance, farad, F, mp_units::si::farad)

UNIT_ADD_CATEGORY_CONCEPT(capacitance, mp_units::isq::capacitance)

using namespace capacitance;

}  // namespace units
