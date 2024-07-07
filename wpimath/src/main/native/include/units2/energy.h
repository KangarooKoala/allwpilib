// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <mp-units/systems/isq/si_quantities.h>

#include "units2/base.h"

namespace units {

UNIT_ALIAS_METRIC(energy, joule, J, mp_units::si::joule)
UNIT_DEFINE_METRIC(energy, calorie, cal, mp_units::mag_ratio<4184, 1000>* joule)
UNIT_DEFINE(energy, kilowatt_hour, kWh, mp_units::mag_ratio<36, 10>* megajoule)
UNIT_DEFINE(energy, watt_hour, Wh, mp_units::mag_ratio<1, 1000>* kilowatt_hour)
UNIT_DEFINE(energy, british_thermal_unit, BTU,
            mp_units::mag_ratio<105'505'585'262, 100'000'000>* joule)
UNIT_DEFINE(energy, british_thermal_unit_iso, BTU_iso,
            mp_units::mag_ratio<1'055'056, 1'000>* joule)
UNIT_DEFINE(energy, british_thermal_unit_59, BTU59,
            mp_units::mag_ratio<1'054'804, 1'000>* joule)
UNIT_DEFINE(energy, therm, thm, mp_units::mag<100'000>* british_thermal_unit_59)
UNIT_DEFINE(
    energy, foot_pound, ftlbf,
    mp_units::mag_ratio<13'558'179'483'314'004, 10'000'000'000'000'000>* joule)

UNIT_ADD_CATEGORY_CONCEPT(energy, mp_units::isq::energy)

using namespace energy;

}  // namespace units
