// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"
#include "units/length.h"

namespace units {

UNIT_DEFINE(volume, cubic_meter, cu_m, mp_units::cubic(meter))
UNIT_DEFINE(volume, cubic_millimeter, cu_mm, mp_units::cubic(millimeter))
UNIT_DEFINE(volume, cubic_kilometer, cu_km, mp_units::cubic(kilometer))
UNIT_ALIAS_METRIC(volume, liter, L, mp_units::si::litre)
UNIT_DEFINE(volume, cubic_inch, cu_in, mp_units::cubic(inch))
UNIT_DEFINE(volume, cubic_foot, cu_ft, mp_units::cubic(foot))
UNIT_DEFINE(volume, cubic_yard, cu_yd, mp_units::cubic(yard))
UNIT_DEFINE(volume, cubic_mile, cu_mi, mp_units::cubic(mile))
UNIT_ALIAS(volume, gallon, gal, mp_units::usc::gallon)
UNIT_ALIAS(volume, quart, qt, mp_units::usc::quart)
UNIT_ALIAS(volume, pint, pt, mp_units::usc::pint)
UNIT_ALIAS(volume, cup, c, mp_units::usc::cup)
UNIT_ALIAS(volume, fluid_ounce, fl_oz, mp_units::usc::fluid_ounce)
UNIT_ALIAS(volume, barrel, bl, mp_units::usc::oil_barrel)
UNIT_ALIAS(volume, bushel, bu, mp_units::usc::bushel)
UNIT_DEFINE(volume, cord, cord, mp_units::mag<128>* cubic_foot)
UNIT_DEFINE(volume, cubic_fathom, cu_fm, mp_units::cubic(fathom))
UNIT_ALIAS(volume, tablespoon, tbsp, mp_units::usc::tablespoon)
UNIT_ALIAS(volume, teaspoon, tsp, mp_units::usc::teaspoon)
UNIT_DEFINE(volume, pinch, pinch, mp_units::mag_ratio<1, 8>* teaspoon)
UNIT_DEFINE(volume, dash, dash, mp_units::mag_ratio<1, 2>* pinch)
UNIT_DEFINE(volume, drop, drop, mp_units::mag_ratio<1, 360>* fluid_ounce)
UNIT_DEFINE(volume, fifth, fifth, mp_units::mag_ratio<1, 5>* gallon)
UNIT_ALIAS(volume, dram, dr, mp_units::usc::fluid_dram)
UNIT_ALIAS(volume, gill, gi, mp_units::usc::gill)
UNIT_ALIAS(volume, peck, pk, mp_units::usc::peck)
UNIT_DEFINE(volume, sack, sacks, mp_units::mag<3>* bushel)
UNIT_ALIAS(volume, shot, shots, mp_units::usc::shot)
UNIT_DEFINE(volume, strike, strikes, mp_units::mag<2>* bushel)

UNIT_ADD_CATEGORY_CONCEPT(volume, mp_units::isq::volume)

using namespace volume;

}  // namespace units
