// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units2/base.h"
#include "units2/length.h"
#include "units2/time.h"

namespace units {

UNIT_DEFINE(acceleration, meters_per_second_squared, mps_sq,
            meter / mp_units::square(second))
UNIT_DEFINE(acceleration, feet_per_second_squared, fps_sq,
            foot / mp_units::square(second))
UNIT_ALIAS(acceleration, standard_gravity, SG, mp_units::si::standard_gravity)

UNIT_ADD_CATEGORY_CONCEPT(acceleration, mp_units::isq::acceleration)

using namespace acceleration;

}  // namespace units
