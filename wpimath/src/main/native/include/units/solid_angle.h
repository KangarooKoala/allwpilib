// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/angle.h"
#include "units/base.h"

namespace units {

UNIT_ALIAS_METRIC(solid_angle, steradian, sr, mp_units::angular::steradian)
UNIT_DEFINE(solid_angle, degree_squared, sq_deg, mp_units::square(degree))
UNIT_DEFINE(solid_angle, spat, sp,
            mp_units::mag<4>* mp_units::mag_pi* steradian)

UNIT_ADD_CATEGORY_CONCEPT(solid_angle, mp_units::angular::solid_angle)

using namespace solid_angle;

}  // namespace units
