// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"

namespace units {

UNIT_ALIAS_METRIC(time, second, s, mp_units::si::second)
UNIT_ALIAS(time, minute, min, mp_units::non_si::minute)
UNIT_ALIAS(time, hour, hr, mp_units::non_si::hour)
UNIT_ALIAS(time, day, d, mp_units::non_si::day)
UNIT_DEFINE(time, week, wk, mp_units::mag<7>* day)
UNIT_DEFINE(time, year, yr, mp_units::mag<365>* day)
UNIT_ALIAS(time, julian_year, a_j, mp_units::iau::Julian_year)
UNIT_DEFINE(time, gregorian_year, a_g, mp_units::mag<31556952>* second)

UNIT_ADD_CATEGORY_CONCEPT(time, mp_units::isq::time)

using namespace time;

}  // namespace units
