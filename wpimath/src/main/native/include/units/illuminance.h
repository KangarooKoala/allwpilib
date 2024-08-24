// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"
#include "units/length.h"
#include "units/luminous_flux.h"

namespace units {

UNIT_ALIAS_METRIC(illuminance, lux, lx, mp_units::si::lux)
UNIT_DEFINE(illuminance, footcandle, fc, lumen / mp_units::square(foot))
UNIT_DEFINE(illuminance, lumens_per_square_inch, lm_per_in_sq,
            lumen / mp_units::square(inch))
UNIT_DEFINE(illuminance, phot, ph, lumen / mp_units::square(centimeter))

UNIT_ADD_CATEGORY_CONCEPT(
    illuminance,
    mp_units::isq::luminous_intensity* mp_units::isq::solid_angular_measure /
        mp_units::isq::area)

using namespace illuminance;

}  // namespace units
