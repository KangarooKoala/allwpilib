// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units2/base.h"
#include "units2/energy.h"
#include "units2/frequency.h"
#include "units2/mass.h"

namespace units {

UNIT_ALIAS_METRIC(radiation, becquerel, Bq, mp_units::si::becquerel)
UNIT_ALIAS_METRIC(radiation, gray, Gy, mp_units::si::gray)
UNIT_ALIAS_METRIC(radiation, sievert, Sv, mp_units::si::sievert)
UNIT_DEFINE(radiation, curie, Ci, mp_units::mag<37>* gigabecquerel)
UNIT_DEFINE(radiation, rutherford, rd, megabecquerel)
UNIT_DEFINE(radiation, rad, rads, centigray)

UNIT_ADD_CATEGORY_CONCEPT(radioactivity, mp_units::isq::activity)

using namespace radiation;

}  // namespace units
