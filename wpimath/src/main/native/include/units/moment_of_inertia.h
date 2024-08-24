// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/area.h"
#include "units/base.h"
#include "units/mass.h"

namespace units {

UNIT_DEFINE(moment_of_inertia, kilogram_square_meter, kg_sq_m,
            kilogram* square_meter)

using namespace moment_of_inertia;

}  // namespace units
