// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/angle.h"
#include "units/base.h"
#include "units/time.h"

namespace units {

UNIT_DEFINE(angular_jerk, radians_per_second_cubed, rad_per_s_cu,
            radian / mp_units::cubic(second))
UNIT_DEFINE(angular_jerk, degrees_per_second_cubed, deg_per_s_cu,
            degree / mp_units::cubic(second))
UNIT_DEFINE(angular_jerk, turns_per_second_cubed, turns_per_s_cu,
            turn / mp_units::cubic(second))

UNIT_ADD_CATEGORY_CONCEPT(angular_jerk,
                          mp_units::angular::angle /
                              mp_units::pow<3>(mp_units::isq::time))

using namespace angular_jerk;

}  // namespace units
