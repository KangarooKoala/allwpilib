// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units2/base.h"
#include "units2/current.h"
#include "units2/time.h"

namespace units {

UNIT_ALIAS_METRIC(charge, coulomb, C, mp_units::si::coulomb)
UNIT_DEFINE_METRIC(charge, ampere_hour, Ah, ampere* hour)

UNIT_ADD_CATEGORY_CONCEPT(charge, mp_units::isq::electric_charge)

using namespace charge;

}  // namespace units
