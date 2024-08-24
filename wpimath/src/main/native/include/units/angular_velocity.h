// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/angle.h"
#include "units/base.h"
#include "units/time.h"

namespace units {

UNIT_DEFINE(angular_velocity, radians_per_second, rad_per_s, radian / second)
UNIT_DEFINE(angular_velocity, degrees_per_second, deg_per_s, degree / second)
UNIT_DEFINE(angular_velocity, turns_per_second, tps, turn / second)
UNIT_DEFINE(angular_velocity, revolutions_per_minute, rpm, turn / minute)
UNIT_DEFINE(angular_velocity, milliarcseconds_per_year, mas_per_yr,
            milliarcsecond / year)

UNIT_ADD_CATEGORY_CONCEPT(angular_velocity,
                          mp_units::angular::angle / mp_units::isq::time)

using namespace angular_velocity;

}  // namespace units
