// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/angle.h"
#include "units/base.h"
#include "units/time.h"

namespace units {

UNIT_DEFINE(angular_acceleration, radians_per_second_squared, rad_per_s_sq,
            radian / mp_units::square(second))
UNIT_DEFINE(angular_acceleration, degrees_per_second_squared, deg_per_s_sq,
            degree / mp_units::square(second))
UNIT_DEFINE(angular_acceleration, revolutions_per_minute_squared, rev_per_m_sq,
            turn / mp_units::square(minute))
UNIT_DEFINE(angular_acceleration, revolutions_per_minute_per_second,
            rev_per_m_per_s, turn / minute / second)

UNIT_ADD_CATEGORY_CONCEPT(angular_acceleration,
                          mp_units::angular::angle /
                              mp_units::pow<2>(mp_units::isq::time))

using namespace angular_acceleration;

}  // namespace units
