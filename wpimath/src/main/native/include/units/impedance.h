// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"

namespace units {

UNIT_ALIAS_METRIC(impedance, ohm, Ohm, mp_units::si::ohm)

UNIT_ADD_CATEGORY_CONCEPT(impedance, mp_units::isq::impedance)

using namespace impedance;

}  // namespace units
