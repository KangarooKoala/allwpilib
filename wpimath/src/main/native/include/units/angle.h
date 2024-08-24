// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"

namespace units {

UNIT_ALIAS_METRIC(angle, radian, rad, mp_units::angular::radian)
UNIT_ALIAS(angle, degree, deg, mp_units::angular::degree)
UNIT_DEFINE(angle, arcminute, arcmin, mp_units::mag_ratio<1, 60>* degree)
UNIT_DEFINE(angle, arcsecond, arcsec, mp_units::mag_ratio<1, 60>* arcminute)
UNIT_DEFINE(angle, milliarcsecond, mas, mp_units::mag_power<10, -3>* arcsecond)
UNIT_DEFINE(angle, turn, tr, mp_units::angular::revolution)
UNIT_DEFINE(angle, gradian, gon, mp_units::mag_ratio<1, 400>* turn)

UNIT_ADD_CATEGORY_CONCEPT(angle, mp_units::angular::angle)

using namespace angle;

}  // namespace units
