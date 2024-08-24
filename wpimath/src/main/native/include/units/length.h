// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/base.h"

namespace units {

UNIT_ALIAS_METRIC(length, meter, m, mp_units::si::metre)
UNIT_ALIAS(length, foot, ft, mp_units::international::foot)
UNIT_ALIAS(length, mil, mil, mp_units::international::mil)
UNIT_ALIAS(length, inch, in, mp_units::international::inch)
UNIT_ALIAS(length, mile, mi, mp_units::international::mile)
UNIT_ALIAS(length, nautical_mile, nmi, mp_units::international::nautical_mile)
UNIT_ALIAS(length, astronomical_unit, au, mp_units::si::astronomical_unit)
UNIT_ALIAS(length, lightyear, ly, mp_units::iau::light_year)
UNIT_ALIAS(length, parsec, pc, mp_units::iau::parsec)
UNIT_ALIAS(length, angstrom, angstrom, mp_units::iau::angstrom)
UNIT_DEFINE(length, cubit, cbt, mp_units::mag<18>* inch)
UNIT_ALIAS(length, fathom, ftm, mp_units::usc::fathom)
UNIT_ALIAS(length, chain, ch, mp_units::imperial::chain)
UNIT_ALIAS(length, furlong, fur, mp_units::imperial::furlong)
UNIT_ALIAS(length, hand, hand, mp_units::imperial::hand)
UNIT_ALIAS(length, league, lea, mp_units::international::league)
UNIT_DEFINE(length, nautical_league, nl, mp_units::mag<3>* nautical_mile)
UNIT_ALIAS(length, yard, yd, mp_units::international::yard)

UNIT_ADD_CATEGORY_CONCEPT(length, mp_units::isq::length)

using namespace length;

}  // namespace units
