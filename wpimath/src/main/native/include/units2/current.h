// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units2/base.h"

namespace units {

UNIT_ALIAS(current, ampere, A, mp_units::si::ampere)

UNIT_ADD_CATEGORY_CONCEPT(current, mp_units::isq::electric_current)

using namespace current;

}  // namespace units
