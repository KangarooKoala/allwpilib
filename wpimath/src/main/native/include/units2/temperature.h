// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units2/base.h"

namespace units {

UNIT_ALIAS_QP(temperature, kelvin, K, K_deg, mp_units::si::kelvin)
UNIT_ALIAS_QP(temperature, celsius, degC, C_deg, mp_units::si::degree_Celsius)
UNIT_ALIAS_QP(temperature, fahrenheit, degF, F_deg,
              mp_units::usc::degree_Fahrenheit)
UNIT_DEFINE_QP(temperature, reaumur, Re, Re_deg,
               mp_units::mag_ratio<5, 4>* celsius)
UNIT_DEFINE_QP(temperature, rankine, Ra, Ra_deg,
               mp_units::mag_ratio<5, 9>* kelvin)

UNIT_ADD_CATEGORY_CONCEPT(temperature, mp_units::isq::thermodynamic_temperature)

using namespace temperature;

}  // namespace units
