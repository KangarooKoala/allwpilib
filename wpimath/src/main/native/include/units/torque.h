// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"
#include "units/energy.h"
#include "units/force.h"
#include "units/length.h"

namespace units {

UNIT_DEFINE(torque, newton_meter, Nm, newton* meter)
UNIT_DEFINE(torque, foot_pound, ftlb, force::pound* foot)
UNIT_DEFINE(torque, foot_poundal, ftpdl, force::poundal* foot)
UNIT_DEFINE(torque, inch_pound, inlb, force::pound* inch)
UNIT_DEFINE(torque, meter_kilogram, mkgf, force::kilopond* meter)

UNIT_ADD_CATEGORY_CONCEPT(torque, mp_units::isq::torque)

using namespace torque;

}  // namespace units
