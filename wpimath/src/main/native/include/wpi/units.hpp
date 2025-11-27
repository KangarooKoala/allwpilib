// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <limits>

#include <gcem.hpp>
#include <mp-units/format.h>
#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/angular.h>
#include <mp-units/systems/si.h>

namespace mp {

using namespace mp_units;

// We can't use the entire angular namespace because we need to define our own
// constexpr trig functions. We can't use the entire si namespace because we
// want to use angular::rad instead of si::rad.

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

// unit name accessors

constexpr auto name(Unit auto u) {
  return fmt::format("{:n}", u);
}

constexpr auto portable_name(Unit auto u) {
  return fmt::format("{:nP}", u);
}

constexpr auto unit_name(Quantity auto q) {
  return name(decltype(q)::unit);
}

constexpr auto portable_unit_name(Quantity auto q) {
  return portable_name(decltype(q)::unit);
}

namespace detail {

// use a wrapper type in a separate namespace so that the ADL picks up the
// constexpr gcem functions instead of the non-constexpr std ones

namespace gcem_adl {

struct wrapped_double {
  double value;

  explicit constexpr wrapped_double(double value) : value{value} {}

  constexpr wrapped_double& operator=(double x) {
    value = x;
    return *this;
  }

  // operator==, operator*, and operator/ are required for
  // mp_units::Representation The other arithmetic operators are defined so that
  // quantity arithmetic inside the mp-units implementations works

  constexpr bool operator==(const wrapped_double&) const = default;

  constexpr wrapped_double operator*(const wrapped_double& rhs) const {
    return wrapped_double{this->value * rhs.value};
  }

  constexpr wrapped_double operator/(const wrapped_double& rhs) const {
    return wrapped_double{this->value / rhs.value};
  }

  constexpr wrapped_double operator+() const {
    return wrapped_double{+this->value};
  }

  constexpr wrapped_double operator-() const {
    return wrapped_double{-this->value};
  }

  constexpr wrapped_double operator+(const wrapped_double& rhs) const {
    return wrapped_double{this->value + rhs.value};
  }

  constexpr wrapped_double operator-(const wrapped_double& rhs) const {
    return wrapped_double{this->value - rhs.value};
  }

  // This needs to be implicit to convert between quantities
  /* implicit */ constexpr operator double() const { return value; }
};

constexpr wrapped_double(pow)(wrapped_double x, double pow) {
  return wrapped_double{gcem::pow(x.value, pow)};
}

constexpr wrapped_double(sqrt)(wrapped_double x) {
  return wrapped_double{gcem::sqrt(x.value)};
}

constexpr wrapped_double(cbrt)(wrapped_double x) {
  return wrapped_double{gcem::pow(x.value, 1.0 / 3.0)};
}

constexpr wrapped_double(exp)(wrapped_double x) {
  return wrapped_double{gcem::exp(x.value)};
}

constexpr wrapped_double(abs)(wrapped_double x) {
  return wrapped_double{gcem::abs(x.value)};
}

constexpr bool isinf(wrapped_double x) {
  constexpr double pos_inf = std::numeric_limits<double>::infinity();
  return x.value == pos_inf || x.value == -pos_inf;
}

constexpr bool isnan(wrapped_double x) {
  return x.value != x.value;
}

constexpr bool isfinite(wrapped_double x) {
  return !isinf(x) && !isnan(x);
}

constexpr wrapped_double fmod(wrapped_double x, wrapped_double y) {
  return wrapped_double{gcem::fmod(x.value, y.value)};
}

constexpr wrapped_double(floor)(wrapped_double x) {
  return wrapped_double{gcem::floor(x.value)};
}

constexpr wrapped_double(ceil)(wrapped_double x) {
  return wrapped_double{gcem::ceil(x.value)};
}

constexpr wrapped_double(round)(wrapped_double x) {
  return wrapped_double{gcem::round(x.value)};
}

constexpr wrapped_double hypot(wrapped_double x, wrapped_double y) {
  return wrapped_double{gcem::hypot(x.value, y.value)};
}

constexpr wrapped_double hypot(wrapped_double x, wrapped_double y,
                               wrapped_double z) {
  return wrapped_double{gcem::hypot(x.value, y.value, z.value)};
}

}  // namespace gcem_adl

template <Reference auto R>
constexpr quantity<R, gcem_adl::wrapped_double> to_gcem_adl(
    quantity<R, double> x) {
  return quantity<R, gcem_adl::wrapped_double>{x};
}

template <Reference auto R>
constexpr quantity<R, double> from_gcem_adl(
    quantity<R, gcem_adl::wrapped_double> x) {
  return quantity<R, double>{x};
}

template <Reference auto R, PointOriginFor<get_quantity_spec(R)> auto PO>
constexpr quantity_point<R, PO, gcem_adl::wrapped_double> to_gcem_adl(
    quantity_point<R, PO, double> x) {
  return quantity_point<R, PO, gcem_adl::wrapped_double>{x};
}

template <Reference auto R, PointOriginFor<get_quantity_spec(R)> auto PO>
constexpr quantity_point<R, PO, double> from_gcem_adl(
    quantity_point<R, PO, gcem_adl::wrapped_double> x) {
  return quantity_point<R, PO, double>{x};
}

}  // namespace detail

}  // namespace mp

template <>
struct std::common_type<intmax_t, mp::detail::gcem_adl::wrapped_double> {
  using type = mp::detail::gcem_adl::wrapped_double;
};

template <>
struct std::common_type<mp::detail::gcem_adl::wrapped_double, intmax_t> {
  using type = mp::detail::gcem_adl::wrapped_double;
};

template <>
inline constexpr bool
    mp_units::treat_as_floating_point<mp::detail::gcem_adl::wrapped_double> =
        true;

template <>
inline constexpr bool
    mp_units::is_scalar<mp::detail::gcem_adl::wrapped_double> = true;

namespace mp {

// gcem versions of the math.h functions which default to the non-constexpr std
// functions; for simplicity, these are hardcoded to only use double as the
// representation type

/**
 * @brief Computes the value of a quantity raised to the `Num/Den` power
 *
 * Both the quantity value and its quantity specification are the base of the
 * operation.
 *
 * @tparam Num Exponent numerator
 * @tparam Den Exponent denominator
 * @param q Quantity being the base of the operation
 * @return Quantity The result of computation
 */
template <intmax_t Num, intmax_t Den = 1, auto R>
  requires mp_units::detail::non_zero<Den>
[[nodiscard]]
constexpr quantity<pow<Num, Den>(R)>(pow)(const quantity<R>& q) noexcept {
  return detail::from_gcem_adl(mp_units::pow<Num, Den>(detail::to_gcem_adl(q)));
}

/**
 * @brief Computes the square root of a quantity
 *
 * Both the quantity value and its quantity specification are the base of the
 * operation.
 *
 * @param q Quantity being the base of the operation
 * @return Quantity The result of computation
 */
template <auto R>
[[nodiscard]]
constexpr quantity<sqrt(R)>(sqrt)(const quantity<R>& q) noexcept {
  return detail::from_gcem_adl(mp_units::sqrt(detail::to_gcem_adl(q)));
}

/**
 * @brief Computes the cubic root of a quantity
 *
 * Both the quantity value and its quantity specification are the base of the
 * operation.
 *
 * @param q Quantity being the base of the operation
 * @return Quantity The result of computation
 */
template <auto R>
[[nodiscard]]
constexpr quantity<cbrt(R)>(cbrt)(const quantity<R>& q) noexcept {
  return detail::from_gcem_adl(mp_units::cbrt(detail::to_gcem_adl(q)));
}

/**
 * @brief Computes Euler's raised to the given power
 *
 * @note Such an operation has sense only for a dimensionless quantity.
 *
 * @param q Quantity being the base of the operation
 * @return Quantity The value of the same quantity type
 */
template <ReferenceOf<dimensionless> auto R>
[[nodiscard]]
constexpr quantity<R>(exp)(const quantity<R>& q) {
  return detail::from_gcem_adl(mp_units::exp(detail::to_gcem_adl(q)));
}

/**
 * @brief Computes the absolute value of a quantity
 *
 * @param q Quantity being the base of the operation
 * @return Quantity The absolute value of a provided quantity
 */
template <auto R>
[[nodiscard]]
constexpr quantity<R>(abs)(const quantity<R>& q) noexcept {
  return detail::from_gcem_adl(mp_units::abs(detail::to_gcem_adl(q)));
}

/**
 * @brief Determines if a quantity is finite.
 *
 * @param a: Quantity to analyze.
 * @return bool: Whether the quantity is finite or not.
 */
template <auto R>
[[nodiscard]]
constexpr bool isfinite(const quantity<R>& a) noexcept {
  return mp_units::isfinite(detail::to_gcem_adl(a));
}

/**
 * @brief Determines if a quantity point is finite.
 *
 * @param a: Quantity point to analyze.
 * @return bool: Whether the quantity point is finite or not.
 */
template <auto R, auto PO>
[[nodiscard]]
constexpr bool isfinite(const quantity_point<R, PO>& a) noexcept {
  return mp_units::isfinite(detail::to_gcem_adl(a));
}

/**
 * @brief Determines if a quantity is infinite.
 *
 * @param a: Quantity to analyze.
 * @return bool: Whether the quantity is infinite or not.
 */
template <auto R>
[[nodiscard]]
constexpr bool isinf(const quantity<R>& a) noexcept {
  return mp_units::isinf(detail::to_gcem_adl(a));
}

/**
 * @brief Determines if a quantity point is infinite.
 *
 * @param a: Quantity point to analyze.
 * @return bool: Whether the quantity point is infinite or not.
 */
template <auto R, auto PO>
[[nodiscard]]
constexpr bool isinf(const quantity_point<R, PO>& a) noexcept {
  return mp::isinf(detail::to_gcem_adl(a));
}

/**
 * @brief Determines if a quantity is a nan.
 *
 * @param a: Quantity to analyze.
 * @return bool: Whether the quantity is a NaN or not.
 */
template <auto R>
[[nodiscard]]
constexpr bool isnan(const quantity<R>& a) noexcept {
  return mp_units::isnan(detail::to_gcem_adl(a));
}

/**
 * @brief Determines if a quantity point is a nan.
 *
 * @param a: Quantity point to analyze.
 * @return bool: Whether the quantity point is a NaN or not.
 */
template <auto R, auto PO>
[[nodiscard]]
constexpr bool isnan(const quantity_point<R, PO>& a) noexcept {
  return mp_units::isnan(detail::to_gcem_adl(a));
}

/**
 * @brief Computes the floating-point remainder of the division operation x / y.
 */
template <auto R1, auto R2>
[[nodiscard]]
constexpr auto fmod(const quantity<R1>& x, const quantity<R2>& y) noexcept {
  return detail::from_gcem_adl(
      mp_units::fmod(detail::to_gcem_adl(x), detail::to_gcem_adl(y)));
}

/**
 * @brief Computes the largest quantity with integer representation and unit
 * type To with its number not greater than q
 *
 * @tparam q Quantity being the base of the operation
 * @return Quantity The rounded quantity with unit type To
 */
template <Unit auto To, auto R>
[[nodiscard]]
constexpr auto(floor)(const quantity<R>& q) noexcept
  requires(equivalent(To, get_unit(R)) || requires { q.force_in(To); })
{
  return detail::from_gcem_adl(mp_units::floor<To>(detail::to_gcem_adl(q)));
}

/**
 * @brief Computes the smallest quantity with integer representation and unit
 * type To with its number not less than q
 *
 * @tparam q Quantity being the base of the operation
 * @return Quantity The rounded quantity with unit type To
 */
template <Unit auto To, auto R>
[[nodiscard]]
constexpr auto(ceil)(const quantity<R>& q) noexcept
  requires(equivalent(To, get_unit(R)) || requires { q.force_in(To); })
{
  return detail::from_gcem_adl(mp_units::ceil<To>(detail::to_gcem_adl(q)));
}

/**
 * @brief Computes the nearest quantity with integer representation and unit
 * type To to q
 *
 * Rounding halfway cases away from zero, regardless of the current rounding
 * mode.
 *
 * @tparam q Quantity being the base of the operation
 * @return Quantity The rounded quantity with unit type To
 */
template <Unit auto To, auto R>
[[nodiscard]]
constexpr auto(round)(const quantity<R>& q) noexcept
  requires(equivalent(To, get_unit(R)) ||
           requires { ::mp_units::floor<To>(q); })
{
  return detail::from_gcem_adl(mp_units::round<To>(detail::to_gcem_adl(q)));
}

/**
 * @brief Computes the square root of the sum of the squares of x and y,
 *        without undue overflow or underflow at intermediate stages of the
 * computation
 */
template <auto R1, auto R2>
[[nodiscard]]
constexpr auto hypot(const quantity<R1>& x, const quantity<R2>& y) noexcept {
  return detail::from_gcem_adl(
      mp_units::hypot(detail::to_gcem_adl(x), detail::to_gcem_adl(y)));
}

/**
 * @brief Computes the square root of the sum of the squares of x, y, and z,
 *        without undue overflow or underflow at intermediate stages of the
 * computation
 */
template <auto R1, auto R2, auto R3>
[[nodiscard]]
constexpr auto hypot(const quantity<R1>& x, const quantity<R2>& y,
                     const quantity<R3>& z) noexcept {
  return detail::from_gcem_adl(mp_units::hypot(
      detail::to_gcem_adl(x), detail::to_gcem_adl(y), detail::to_gcem_adl(z)));
}

// misc. math functions (not included by math.h)
template <ReferenceOf<dimensionless> auto R>
[[nodiscard]]
inline constexpr quantity<R>(log)(const quantity<R>& q) noexcept {
  // based on the mp_units::exp implementation
  return value_cast<get_unit(R)>(
      quantity{gcem::log(q.force_numerical_value_in(q.unit)),
               mp_units::detail::clone_reference_with<one>(R)});
}

template <auto R1, auto R2>
  requires requires { get_common_reference(R1, R2); }
[[nodiscard]]
constexpr QuantityOf<get_quantity_spec(R1)> auto copysign(
    const quantity<R1>& x, const quantity<R2>& y) noexcept {
  // based on the mp_units::fmod implementation
  constexpr auto ref = get_common_reference(R1, R2);
  constexpr auto unit = mp_units::get_unit(ref);
  return quantity{gcem::copysign(x.numerical_value_ref_in(unit),
                                 y.numerical_value_ref_in(unit)),
                  ref};
}

template <auto R>
[[nodiscard]]
constexpr quantity<R>(copysign)(const quantity<R>& x, double y) noexcept {
  // based on the mp_units::abs implementation
  return {gcem::copysign(x.numerical_value_ref_in(x.unit), y), R};
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

template <auto R1, auto R2>
  requires requires { get_common_reference(R1, R2); }
[[nodiscard]]
inline constexpr quantity<rad>(atan2)(quantity<R1> y, quantity<R2> x) noexcept {
  constexpr auto ref = get_common_reference(R1, R2);
  constexpr auto unit = get_unit(ref);
  return gcem::atan2(value(y.in(unit)), value(x.in(unit))) * rad;
}

// base unit
consteval Unit auto get_base_unit(Unit auto u) {
  return get_canonical_unit(u).reference_unit;
}

}  // namespace mp

namespace wpi::util {

template <mp::Quantity T>
constexpr int sgn(T val) {
  return (T::zero() < val) - (val < T::zero());
}

}  // namespace wpi::util
