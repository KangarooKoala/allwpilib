// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units2/base.h"
#include "units2/length.h"
#include "units2/time.h"

namespace units {

UNIT_DEFINE(velocity, meters_per_second, mps, meter / second)
UNIT_DEFINE(velocity, feet_per_second, fps, foot / second)
UNIT_DEFINE(velocity, miles_per_hour, mph, mile / hour)
UNIT_DEFINE(velocity, kilometers_per_hour, kph, kilometer / hour)
UNIT_DEFINE(velocity, knot, kts, nautical_mile / hour)

UNIT_ADD_CATEGORY_CONCEPT(velocity, mp_units::isq::velocity)

using namespace velocity;

}  // namespace units
