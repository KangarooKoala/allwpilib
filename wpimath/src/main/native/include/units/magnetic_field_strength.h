// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"
#include "units/length.h"
#include "units/magnetic_flux.h"

namespace units {

// Unfortunately `_T` is a WINAPI macro, so we have to use `_Te` as the tesla
// abbreviation.
UNIT_ALIAS_METRIC(magnetic_field_strength, tesla, Te, mp_units::si::tesla)
UNIT_DEFINE(magnetic_field_strength, gauss, G,
            maxwell / mp_units::square(centimeter))

UNIT_ADD_CATEGORY_CONCEPT(magnetic_field_strength,
                          mp_units::isq::magnetic_flux_density)

using namespace magnetic_field_strength;

}  // namespace units
