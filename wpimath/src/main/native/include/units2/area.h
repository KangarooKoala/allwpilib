// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units2/base.h"
#include "units2/length.h"

namespace units {

UNIT_DEFINE(area, square_meter, sq_m, mp_units::square(meter))
UNIT_DEFINE(area, square_foot, sq_ft, mp_units::square(foot))
UNIT_DEFINE(area, square_inch, sq_in, mp_units::square(inch))
UNIT_DEFINE(area, square_mile, sq_mi, mp_units::square(mile))
UNIT_DEFINE(area, square_kilometer, sq_km, mp_units::square(kilometer))
UNIT_ALIAS(area, hectare, ha, mp_units::non_si::hectare)
UNIT_ALIAS(area, acre, acre, mp_units::imperial::acre)

UNIT_ADD_CATEGORY_CONCEPT(area, mp_units::isq::area)

using namespace area;

}  // namespace units
