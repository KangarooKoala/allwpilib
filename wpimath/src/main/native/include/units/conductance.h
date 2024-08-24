// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"

namespace units {

UNIT_ALIAS(conductance, siemens, S, mp_units::si::siemens)

UNIT_ADD_CATEGORY_CONCEPT(conductance, mp_units::isq::conductance)

using namespace conductance;

}  // namespace units
