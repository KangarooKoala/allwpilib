// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"
#include "units/mass.h"
#include "units/volume.h"

namespace units {

UNIT_DEFINE(density, kilograms_per_cubic_meter, kg_per_cu_m,
            kilogram / cubic_meter)
UNIT_DEFINE(density, grams_per_milliliter, g_per_mL, gram / milliliter)
UNIT_DEFINE(density, kilogram_per_liter, kg_per_L, kilogram / milliliter)
UNIT_DEFINE(density, ounces_per_cubic_foot, oz_per_cu_ft, ounce / cubic_foot)
UNIT_DEFINE(density, ounces_per_cubic_inch, oz_per_cu_in, ounce / cubic_inch)
UNIT_DEFINE(density, ounces_per_gallon, oz_per_gal, ounce / gallon)
UNIT_DEFINE(density, pounds_per_cubic_foot, lb_per_cu_ft, pound / cubic_foot)
UNIT_DEFINE(density, pounds_per_cubic_in, lb_per_cu_in, pound / cubic_inch)
UNIT_DEFINE(density, pounds_per_gallon, lb_per_gal, pound / gallon)
UNIT_DEFINE(density, slugs_per_cubic_foot, slugs_per_cubic_foot,
            slug / cubic_foot)

UNIT_ADD_CATEGORY_CONCEPT(density, mp_units::isq::mass_density)

using namespace density;

}  // namespace units
