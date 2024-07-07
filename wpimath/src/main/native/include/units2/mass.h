// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units2/base.h"

namespace units {

UNIT_ALIAS_METRIC(mass, gram, g, mp_units::si::gram)
UNIT_DEFINE(mass, metric_ton, t, mp_units::mag<1000>* kilogram)
UNIT_ALIAS(mass, pound, lb, mp_units::international::pound)
UNIT_ALIAS(mass, long_ton, ln_t, mp_units::imperial::long_ton)
UNIT_ALIAS(mass, short_ton, sh_t, mp_units::usc::short_ton)
UNIT_ALIAS(mass, stone, st, mp_units::imperial::stone)
UNIT_ALIAS(mass, ounce, oz, mp_units::international::ounce)
UNIT_DEFINE(mass, carat, ct, mp_units::mag<200>* milligram)
UNIT_DEFINE(mass, slug, slug,
            mp_units::mag_ratio<145'939'029, 10'000'000>* kilogram)

UNIT_ADD_CATEGORY_CONCEPT(mass, mp_units::isq::mass)

using namespace mass;

}  // namespace units
