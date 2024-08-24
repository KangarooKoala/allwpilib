// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/acceleration.h"
#include "units/base.h"
#include "units/length.h"
#include "units/mass.h"
#include "units/time.h"

namespace units {

UNIT_ALIAS_METRIC(force, newton, N, mp_units::si::newton)
UNIT_DEFINE(force, pound, lbf, slug* foot / mp_units::square(second))
UNIT_ALIAS(force, dyne, dyn, mp_units::cgs::dyne)
UNIT_DEFINE(force, kilopond, kp, standard_gravity* kilogram)
UNIT_ALIAS(force, poundal, pdl, mp_units::international::poundal)

UNIT_ADD_CATEGORY_CONCEPT(force, mp_units::isq::force)

using force::newton;
using force::newton_t;

}  // namespace units
