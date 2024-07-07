// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units2/base.h"

namespace units {

UNIT_ALIAS_METRIC(luminous_flux, lumen, lm, mp_units::si::lumen)

UNIT_ADD_CATEGORY_CONCEPT(
    luminous_flux,
    mp_units::isq::luminous_intensity* mp_units::isq::solid_angular_measure)

using namespace luminous_flux;

}  // namespace units
