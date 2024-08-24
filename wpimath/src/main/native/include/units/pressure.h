// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"
#include "units/force.h"
#include "units/length.h"

namespace units {

UNIT_ALIAS_METRIC(pressure, pascal, Pa, mp_units::si::pascal)
UNIT_DEFINE(pressure, bar, bar, mp_units::mag<100>* kilopascal)
UNIT_ALIAS(pressure, mbar, mbar, mp_units::si::milli<bar>)
UNIT_DEFINE(pressure, atmosphere, atm, mp_units::mag<101'325>* pascal)
UNIT_DEFINE(pressure, pounds_per_square_inch, psi,
            force::pound / mp_units::square(inch))
UNIT_DEFINE(pressure, torr, torr, mp_units::mag_ratio<1, 760>* atmosphere)

UNIT_ADD_CATEGORY_CONCEPT(pressure, mp_units::isq::pressure)

using namespace pressure;

}  // namespace units
