// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <gcem.hpp>

#include "mp-units/framework.h"
#include "mp-units/math.h"
#include "mp-units/systems/angular.h"
#include "mp-units/systems/si.h"

namespace mp {

using namespace mp_units;

// Selectively pull in items from angular to let us define our own constexpr
// trig functions Selectively pull in items from si to use angular::rad instead
// of si::rad

// angular units
using angular::degree;
using angular::gradian;
using angular::radian;
using angular::revolution;
using angular::steradian;

// SI base units
using si::ampere;
using si::candela;
using si::gram;
using si::kelvin;
using si::kilogram;
using si::metre;
using si::mole;
using si::second;

// SI derived named units
using si::becquerel;
using si::coulomb;
using si::degree_Celsius;
using si::farad;
using si::gray;
using si::henry;
using si::hertz;
using si::joule;
using si::katal;
using si::lumen;
using si::lux;
using si::newton;
using si::ohm;
using si::pascal;
using si::siemens;
using si::sievert;
using si::tesla;
using si::volt;
using si::watt;
using si::weber;

// non-SI units
using non_si::are;
using non_si::astronomical_unit;
using non_si::dalton;
using non_si::day;
using non_si::electronvolt;
using non_si::hectare;
using non_si::hour;
using non_si::litre;
using non_si::minute;
using non_si::tonne;

namespace unit_symbols {

// angular unit symbols
using angular::unit_symbols::deg;
using angular::unit_symbols::deg2;
using angular::unit_symbols::grad;
using angular::unit_symbols::rad;
using angular::unit_symbols::rad2;
using angular::unit_symbols::rev;
using angular::unit_symbols::sr;

// SI unit symbols
#define ADD_METRIC_UNIT_SYMBOLS_NO_MICRO(namespaceName, symbolName) \
  using namespaceName::unit_symbols::q##symbolName;                 \
  using namespaceName::unit_symbols::r##symbolName;                 \
  using namespaceName::unit_symbols::y##symbolName;                 \
  using namespaceName::unit_symbols::z##symbolName;                 \
  using namespaceName::unit_symbols::a##symbolName;                 \
  using namespaceName::unit_symbols::f##symbolName;                 \
  using namespaceName::unit_symbols::p##symbolName;                 \
  using namespaceName::unit_symbols::n##symbolName;                 \
  using namespaceName::unit_symbols::m##symbolName;                 \
  using namespaceName::unit_symbols::c##symbolName;                 \
  using namespaceName::unit_symbols::d##symbolName;                 \
  using namespaceName::unit_symbols::symbolName;                    \
  using namespaceName::unit_symbols::da##symbolName;                \
  using namespaceName::unit_symbols::h##symbolName;                 \
  using namespaceName::unit_symbols::k##symbolName;                 \
  using namespaceName::unit_symbols::M##symbolName;                 \
  using namespaceName::unit_symbols::G##symbolName;                 \
  using namespaceName::unit_symbols::T##symbolName;                 \
  using namespaceName::unit_symbols::P##symbolName;                 \
  using namespaceName::unit_symbols::E##symbolName;                 \
  using namespaceName::unit_symbols::Z##symbolName;                 \
  using namespaceName::unit_symbols::Y##symbolName;                 \
  using namespaceName::unit_symbols::R##symbolName;                 \
  using namespaceName::unit_symbols::Q##symbolName;

#define ADD_METRIC_UNIT_SYMBOLS(namespaceName, symbolName)    \
  ADD_METRIC_UNIT_SYMBOLS_NO_MICRO(namespaceName, symbolName) \
  using namespaceName::unit_symbols::u##symbolName;           \
  using namespaceName::unit_symbols::µ##symbolName;

// SI units with metric prefixes (except ones in angular)
ADD_METRIC_UNIT_SYMBOLS(si, m)
ADD_METRIC_UNIT_SYMBOLS(si, s)
ADD_METRIC_UNIT_SYMBOLS(si, g)
ADD_METRIC_UNIT_SYMBOLS(si, A)
ADD_METRIC_UNIT_SYMBOLS(si, K)
ADD_METRIC_UNIT_SYMBOLS(si, mol)
ADD_METRIC_UNIT_SYMBOLS(si, cd)
ADD_METRIC_UNIT_SYMBOLS(si, Hz)
ADD_METRIC_UNIT_SYMBOLS(si, N)
ADD_METRIC_UNIT_SYMBOLS(si, Pa)
ADD_METRIC_UNIT_SYMBOLS(si, J)
ADD_METRIC_UNIT_SYMBOLS(si, W)
ADD_METRIC_UNIT_SYMBOLS(si, C)
ADD_METRIC_UNIT_SYMBOLS(si, V)
ADD_METRIC_UNIT_SYMBOLS(si, F)
ADD_METRIC_UNIT_SYMBOLS_NO_MICRO(si, ohm)
using si::unit_symbols::uohm;
ADD_METRIC_UNIT_SYMBOLS_NO_MICRO(si, Ω)
using si::unit_symbols::µΩ;
ADD_METRIC_UNIT_SYMBOLS(si, S)
ADD_METRIC_UNIT_SYMBOLS(si, Wb)
ADD_METRIC_UNIT_SYMBOLS(si, T)
ADD_METRIC_UNIT_SYMBOLS(si, H)
ADD_METRIC_UNIT_SYMBOLS(si, lm)
ADD_METRIC_UNIT_SYMBOLS(si, lx)
ADD_METRIC_UNIT_SYMBOLS(si, Bq)
ADD_METRIC_UNIT_SYMBOLS(si, Gy)
ADD_METRIC_UNIT_SYMBOLS(si, Sv)
ADD_METRIC_UNIT_SYMBOLS(si, kat)

#undef ADD_METRIC_UNIT_SYMBOLS

// SI units without prefixes
using si::unit_symbols::deg_C;
using si::unit_symbols::m2;
using si::unit_symbols::m3;
using si::unit_symbols::m4;
using si::unit_symbols::s2;
using si::unit_symbols::s3;

// non-SI units
using non_si::unit_symbols::a;
using non_si::unit_symbols::au;
using non_si::unit_symbols::d;
using non_si::unit_symbols::Da;
using non_si::unit_symbols::eV;
using non_si::unit_symbols::h;  // Should we use hr as well?
using non_si::unit_symbols::ha;
using non_si::unit_symbols::l;
using non_si::unit_symbols::L;
using non_si::unit_symbols::min;
using non_si::unit_symbols::t;

}  // namespace unit_symbols

using namespace unit_symbols;

// quantity specs
using angular::angle;
using angular::solid_angle;
using namespace isq;

// value accessor
constexpr double value(Quantity auto q) {
  return q.numerical_value_in(decltype(q)::unit);
}

// trig functions
[[nodiscard]]
inline constexpr double sin(quantity<rad> q) noexcept {
  return gcem::sin(value(q));
}

[[nodiscard]]
inline constexpr double cos(quantity<rad> q) noexcept {
  return gcem::cos(value(q));
}

[[nodiscard]]
inline constexpr double tan(quantity<rad> q) noexcept {
  return gcem::tan(value(q));
}

[[nodiscard]]
inline constexpr quantity<rad>(asin)(quantity<one> q) noexcept {
  return gcem::asin(value(q)) * rad;
}

[[nodiscard]]
inline constexpr quantity<rad>(acos)(quantity<one> q) noexcept {
  return gcem::acos(value(q)) * rad;
}

[[nodiscard]]
inline constexpr quantity<rad>(atan)(quantity<one> q) noexcept {
  return gcem::atan(value(q)) * rad;
}

template <auto R1, typename Rep1, auto R2, typename Rep2>
  requires requires { get_common_reference(R1, R2); }
[[nodiscard]]
inline constexpr quantity<rad>(atan2)(quantity<R1, Rep1> y,
                                      quantity<R2, Rep2> x) noexcept {
  constexpr auto ref = get_common_reference(R1, R2);
  constexpr auto unit = get_unit(ref);
  return gcem::atan2(value(y.in(unit)), value(x.in(unit))) * rad;
}

// base unit
consteval Unit auto get_base_unit(Unit auto u) {
  return get_canonical_unit(u).reference_unit;
}

}  // namespace mp

namespace wpi {

template <mp::Quantity T>
constexpr int sgn(T val) {
  return (T::zero() < val) - (val < T::zero());
}

}  // namespace wpi
