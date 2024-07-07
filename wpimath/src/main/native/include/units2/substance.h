// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units2/base.h"

namespace units {

UNIT_ALIAS(substance, mole, mol, mp_units::si::mole)

UNIT_ADD_CATEGORY_CONCEPT(substance, mp_units::isq::amount_of_substance)

using namespace substance;

}  // namespace units
