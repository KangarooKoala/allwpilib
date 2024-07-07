// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <mp-units/compat_macros.h>
#include <mp-units/ext/format.h>
#include <mp-units/format.h>
#include <mp-units/math.h>
#include <mp-units/ostream.h>
#include <mp-units/systems/angular.h>
#include <mp-units/systems/cgs.h>
#include <mp-units/systems/iau.h>
#include <mp-units/systems/iec80000.h>
#include <mp-units/systems/imperial.h>
#include <mp-units/systems/international.h>
#include <mp-units/systems/isq.h>
#include <mp-units/systems/si.h>
#include <mp-units/systems/usc.h>

#include <gcem.hpp>

#define UNIT_ADD_LITERALS(namespaceName, name, abbreviation)          \
  namespace literals {                                                \
  inline constexpr namespaceName::name##_t operator""_##abbreviation( \
      long double d) {                                                \
    return d * namespaceName::name;                                   \
  }                                                                   \
  inline constexpr namespaceName::name##_t operator""_##abbreviation( \
      unsigned long long int d) {                                     \
    return d * namespaceName::name;                                   \
  }                                                                   \
  }  /* namespace literals */

#define UNIT_ADD_QUANTITY_POINT_LITERALS(namespaceName, name, abbreviation) \
  namespace literals {                                                      \
  inline constexpr mp_units::quantity_point<namespaceName::name>            \
  operator""_##abbreviation(long double d) {                                \
    return mp_units::quantity_point{d * namespaceName::name};               \
  }                                                                         \
  inline constexpr mp_units::quantity_point<namespaceName::name>            \
  operator""_##abbreviation(unsigned long long int d) {                     \
    return mp_units::quantity_point{d * namespaceName::name};               \
  }                                                                         \
  }  /* namespace literals */

#define UNIT_DEFINE_NAMED_UNIT(name, abbreviation, ...)    \
  inline constexpr struct name final                       \
      : mp_units::named_unit<#abbreviation, __VA_ARGS__> { \
  } name;

#define UNIT_DEFINE(namespaceName, name, abbreviation, ...) \
  namespace namespaceName {                                 \
  UNIT_DEFINE_NAMED_UNIT(name, abbreviation, __VA_ARGS__)   \
  using name##_t = mp_units::quantity<name>;                \
  }  /* namespace namespaceName */                          \
  UNIT_ADD_LITERALS(namespaceName, name, abbreviation)

#define UNIT_ALIAS(namespaceName, name, abbreviation, ...) \
  namespace namespaceName {                                \
  inline constexpr auto name = __VA_ARGS__;                \
  using name##_t = mp_units::quantity<name>;               \
  }  /* namespace namespaceName */                         \
  UNIT_ADD_LITERALS(namespaceName, name, abbreviation)

#define UNIT_DEFINE_QP(namespaceName, name, qp_abbreviation, q_abbreviation, \
                       ...)                                                  \
  UNIT_DEFINE(namespaceName, name, q_abbreviation, __VA_ARGS__)              \
  UNIT_ADD_QUANTITY_POINT_LITERALS(namespaceName, name, qp_abbreviation)

#define UNIT_ALIAS_QP(namespaceName, name, qp_abbreviation, q_abbreviation, \
                      ...)                                                  \
  UNIT_ALIAS(namespaceName, name, q_abbreviation, __VA_ARGS__)              \
  UNIT_ADD_QUANTITY_POINT_LITERALS(namespaceName, name, qp_abbreviation)

#define UNIT_ADD_METRIC(type, namespaceName, name, abbreviation, ...) \
  UNIT_##type(namespaceName, name, abbreviation, __VA_ARGS__)         \
  UNIT_ALIAS(namespaceName, quecto##name, q##abbreviation,            \
             mp_units::si::quecto<name>)                              \
  UNIT_ALIAS(namespaceName, ronto##name, r##abbreviation,             \
             mp_units::si::ronto<name>)                               \
  UNIT_ALIAS(namespaceName, yocto##name, y##abbreviation,             \
             mp_units::si::yocto<name>)                               \
  UNIT_ALIAS(namespaceName, zepto##name, z##abbreviation,             \
             mp_units::si::zepto<name>)                               \
  UNIT_ALIAS(namespaceName, atto##name, a##abbreviation,              \
            mp_units::si::atto<name>)                                 \
  UNIT_ALIAS(namespaceName, femto##name, f##abbreviation,             \
             mp_units::si::femto<name>)                               \
  UNIT_ALIAS(namespaceName, pico##name, p##abbreviation,              \
             mp_units::si::pico<name>)                                \
  UNIT_ALIAS(namespaceName, nano##name, n##abbreviation,              \
             mp_units::si::nano<name>)                                \
  UNIT_ALIAS(namespaceName, micro##name, u##abbreviation,             \
             mp_units::si::micro<name>)                               \
  UNIT_ALIAS(namespaceName, milli##name, m##abbreviation,             \
             mp_units::si::milli<name>)                               \
  UNIT_ALIAS(namespaceName, centi##name, c##abbreviation,             \
             mp_units::si::centi<name>)                               \
  UNIT_ALIAS(namespaceName, deci##name, d##abbreviation,              \
             mp_units::si::deci<name>)                                \
  UNIT_ALIAS(namespaceName, deca##name, da##abbreviation,             \
             mp_units::si::deca<name>)                                \
  UNIT_ALIAS(namespaceName, hecto##name, h##abbreviation,             \
             mp_units::si::hecto<name>)                               \
  UNIT_ALIAS(namespaceName, kilo##name, k##abbreviation,              \
             mp_units::si::kilo<name>)                                \
  UNIT_ALIAS(namespaceName, mega##name, M##abbreviation,              \
             mp_units::si::mega<name>)                                \
  UNIT_ALIAS(namespaceName, giga##name, G##abbreviation,              \
             mp_units::si::giga<name>)                                \
  UNIT_ALIAS(namespaceName, tera##name, T##abbreviation,              \
             mp_units::si::tera<name>)                                \
  UNIT_ALIAS(namespaceName, peta##name, P##abbreviation,              \
             mp_units::si::peta<name>)                                \
  UNIT_ALIAS(namespaceName, exa##name, E##abbreviation,               \
             mp_units::si::exa<name>)                                 \
  UNIT_ALIAS(namespaceName, zetta##name, Z##abbreviation,             \
             mp_units::si::zetta<name>)                               \
  UNIT_ALIAS(namespaceName, yotta##name, Y##abbreviation,             \
             mp_units::si::yotta<name>)                               \
  UNIT_ALIAS(namespaceName, ronna##name, R##abbreviation,             \
             mp_units::si::ronna<name>)                               \
  UNIT_ALIAS(namespaceName, quetta##name, Q##abbreviation,            \
             mp_units::si::quetta<name>)

#define UNIT_DEFINE_METRIC(namespaceName, name, abbreviation, ...) \
  UNIT_ADD_METRIC(DEFINE, namespaceName, name, abbreviation, __VA_ARGS__)

#define UNIT_ALIAS_METRIC(namespaceName, name, abbreviation, ...) \
  UNIT_ADD_METRIC(ALIAS, namespaceName, name, abbreviation, __VA_ARGS__)

#define UNIT_ADD_LARGE_METRIC_AND_BINARY(type, namespaceName, name, \
                                         abbreviation, ...)         \
  UNIT_##type(namespaceName, name, abbreviation, __VA_ARGS__)       \
  UNIT_ALIAS(namespaceName, kilo##name, k##abbreviation,            \
             mp_units::si::kilo<name>)                              \
  UNIT_ALIAS(namespaceName, mega##name, M##abbreviation,            \
             mp_units::si::mega<name>)                              \
  UNIT_ALIAS(namespaceName, giga##name, G##abbreviation,            \
             mp_units::si::giga<name>)                              \
  UNIT_ALIAS(namespaceName, tera##name, T##abbreviation,            \
             mp_units::si::tera<name>)                              \
  UNIT_ALIAS(namespaceName, peta##name, P##abbreviation,            \
             mp_units::si::peta<name>)                              \
  UNIT_ALIAS(namespaceName, exa##name, E##abbreviation,             \
             mp_units::si::exa<name>)                               \
  UNIT_ALIAS(namespaceName, zetta##name, Z##abbreviation,           \
             mp_units::si::zetta<name>)                             \
  UNIT_ALIAS(namespaceName, yotta##name, Y##abbreviation,           \
             mp_units::si::yotta<name>)                             \
  UNIT_ALIAS(namespaceName, ronna##name, R##abbreviation,           \
             mp_units::si::ronna<name>)                             \
  UNIT_ALIAS(namespaceName, quetta##name, Q##abbreviation,          \
             mp_units::si::quetta<name>)                            \
  UNIT_ALIAS(namespaceName, kibi##name, Ki##abbreviation,           \
             mp_units::iec80000::kibi<name>)                        \
  UNIT_ALIAS(namespaceName, mebi##name, Mi##abbreviation,           \
             mp_units::iec80000::mebi<name>)                        \
  UNIT_ALIAS(namespaceName, tebi##name, Ti##abbreviation,           \
             mp_units::iec80000::tebi<name>)                        \
  UNIT_ALIAS(namespaceName, pebi##name, Pi##abbreviation,           \
             mp_units::iec80000::pebi<name>)                        \
  UNIT_ALIAS(namespaceName, exbi##name, Ei##abbreviation,           \
             mp_units::iec80000::exbi<name>)

#define UNIT_DEFINE_LARGE_METRIC_AND_BINARY(namespaceName, name, abbreviation, \
                                            ...)                               \
  UNIT_ADD_LARGE_METRIC_AND_BINARY(DEFINE, namespaceName, name, abbreviation,  \
                                   __VA_ARGS__)

#define UNIT_ALIAS_LARGE_METRIC_AND_BINARY(namespaceName, name, abbreviation, \
                                           ...)                               \
  UNIT_ADD_LARGE_METRIC_AND_BINARY(ALIAS, namespaceName, name, abbreviation,  \
                                   __VA_ARGS__)

// TODO Upper camel case version?
#define UNIT_ADD_CATEGORY_CONCEPT(categoryName, quantityKind) \
  template <typename T>                                       \
  concept categoryName##_unit = mp_units::QuantityOf<T, quantityKind>;

namespace units {

template <mp_units::Reference auto R>
using unit_t = mp_units::quantity<R, double>;

template <typename T>
concept UnitT = mp_units::Quantity<T>;

namespace math {

template <int power, UnitT UnitType>
inline constexpr unit_t<mp_units::pow<power>(UnitType::reference)> pow(
    const UnitType& value) noexcept {
  return {gcem::pow(value.value(), power),
          mp_units::pow<power>(UnitType::reference)};
}

template <int power, UnitT UnitType>
inline constexpr unit_t<mp_units::pow<power>(UnitType::reference)> cpow(
    const UnitType& value) noexcept {
  return {gcem::pow(value.value(), power),
          mp_units::pow<power>(UnitType::reference)};
}

}  // namespace math

namespace dimensionless {

inline constexpr auto scalar = mp_units::one;
inline constexpr auto dimensionless = mp_units::one;
using scalar_t = mp_units::quantity<scalar>;
using dimensionless_t = mp_units::quantity<dimensionless>;

}  // namespace dimensionless

UNIT_ADD_CATEGORY_CONCEPT(dimensionless, mp_units::dimensionless)

}  // namespace units

namespace units::literals {}

using namespace units::literals;
