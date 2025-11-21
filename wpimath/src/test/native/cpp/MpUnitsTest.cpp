// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "wpi/units.hpp"

static_assert(mp::abs(mp::pow<2>(2.0 * mp::m) - 4.0 * mp::m2) < 1e-9 * mp::m2);
static_assert(mp::sqrt(9.0 * mp::s2) == 3.0 * mp::s);
static_assert(mp::cbrt(8.0 * mp::m3) == 2.0 * mp::m);
static_assert(mp::exp(0.0 * mp::m / mp::m) == 1.0 * mp::m / mp::m);
static_assert(mp::abs(-1.0 * mp::rad) == 1.0 * mp::rad);
static_assert(mp::isfinite(1.0 * mp::s));
static_assert(!mp::isinf(1.0 * mp::s));
static_assert(!mp::isnan(1.0 * mp::s));
static_assert(mp::fmod(1.5 * mp::m, 1.0 * mp::m) == 0.5 * mp::m);
static_assert(mp::floor<mp::m>(1.75 * mp::m) == 1.0 * mp::m);
static_assert(mp::floor<mp::cm>(1.75 * mp::m) == 175.0 * mp::cm);
static_assert(mp::ceil<mp::m>(1.25 * mp::m) == 2.0 * mp::m);
static_assert(mp::ceil<mp::cm>(1.25 * mp::m) == 125.0 * mp::cm);
static_assert(mp::round<mp::m>(1.25 * mp::m) == 1.0 * mp::m);
static_assert(mp::round<mp::m>(0.75 * mp::m) == 1.0 * mp::m);
static_assert(mp::round<mp::cm>(1.25 * mp::m) == 125.0 * mp::cm);
static_assert(mp::hypot(3.0 * mp::m, 4.0 * mp::m) == 5.0 * mp::m);
static_assert(mp::hypot(1.0 * mp::m, 2.0 * mp::m, 2.0 * mp::m) == 3.0 * mp::m);
static_assert(mp::log(mp::exp(1.0 * mp::m / mp::m)) == 1.0 * mp::m / mp::m);

using namespace std::literals::string_view_literals;

TEST(MpUnits, UnitName) {
  EXPECT_EQ("m s⁻¹"sv, mp::unit_name(2.0 * mp::m / mp::s));
}
