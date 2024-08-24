// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"

namespace units {

UNIT_ALIAS_METRIC(magnetic_flux, weber, Wb, mp_units::si::weber)
UNIT_DEFINE(magnetic_flux, maxwell, Mx, mp_units::mag_power<10, -8>* weber)

UNIT_ADD_CATEGORY_CONCEPT(magnetic_flux, mp_units::isq::magnetic_flux)

using namespace magnetic_flux;

}  // namespace units
