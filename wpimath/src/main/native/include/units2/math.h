// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>

#include <gcem.hpp>

#include "units2/angle.h"
#include "units2/base.h"
#include "units2/dimensionless.h"

/**
 * @brief namespace for unit-enabled versions of the `<cmath>` library
 * @details Includes trigonometric functions, exponential/log functions,
 *          rounding functions, etc.
 */
namespace units::math {

//----------------------------------
// MIN/MAX FUNCTIONS
//----------------------------------

template <UnitT UnitTypeLhs, UnitT UnitTypeRhs>
  requires std::is_convertible_v<UnitTypeRhs, UnitTypeLhs>
constexpr UnitTypeLhs min(const UnitTypeLhs& lhs, const UnitTypeRhs& rhs) {
  UnitTypeLhs r{rhs};
  return lhs < r ? lhs : r;
}

template <UnitT UnitTypeLhs, UnitT UnitTypeRhs>
  requires std::is_convertible_v<UnitTypeRhs, UnitTypeLhs>
constexpr UnitTypeLhs max(const UnitTypeLhs& lhs, const UnitTypeRhs& rhs) {
  UnitTypeLhs r{rhs};
  return lhs > r ? lhs : r;
}

//----------------------------------
// TRIGONOMETRIC FUNCTIONS
//----------------------------------

/**
 * @ingroup UnitMath
 * @brief Compute cosine
 * @details The input value can be in any unit of angle, including radians or
 *          degrees.
 * @tparam AngleUnit any angle `quantity` type.
 * @param[in] angle angle to compute the cosine of
 * @returns Returns the cosine of <i>angle</i>
 */
template <angle_unit AngleUnit>
constexpr scalar_t cos(AngleUnit angle) noexcept {
  return gcem::cos(radian_t{angle}.value()) * scalar;
}

/**
 * @ingroup UnitMath
 * @brief Compute sine
 * @details The input value can be in any unit of angle, including radians or
 *          degrees.
 * @tparam AngleUnit  any angle `quantity` type.
 * @param[in] angle angle to compute the since of
 * @returns Returns the sine of <i>angle</i>
 */
template <angle_unit AngleUnit>
constexpr scalar_t sin(AngleUnit angle) noexcept {
  return gcem::sin(radian_t{angle}.value()) * scalar;
}

/**
 * @ingroup UnitMath
 * @brief Compute tangent
 * @details The input value can be in any unit of angle, including radians or
 *          degrees.
 * @tparam AngleUnit  any angle `quantity` type.
 * @param[in] angle angle to compute the tangent of
 * @returns Returns the tangent of <i>angle</i>
 */
template <angle_unit AngleUnit>
constexpr scalar_t tan(AngleUnit angle) noexcept {
  return gcem::tan(radian_t{angle}.value()) * scalar;
}

/**
 * @ingroup UnitMath
 * @brief Compute arc cosine
 * @details Returns the principal value of the arc cosine of x, expressed in
 *          radians.
 * @param[in] x Value whose arc cosine is computed, in the interval [-1,+1].
 * @returns Principal arc cosine of x, in the interval [0,pi] radians.
 */
template <dimensionless_unit ScalarUnit>
constexpr radian_t acos(ScalarUnit x) noexcept {
  return gcem::acos(x.value()) * radian;
}

/**
 * @ingroup UnitMath
 * @brief Compute arc sine
 * @details Returns the principal value of the arc sine of x, expressed in
 *          radians.
 * @param[in] x Value whose arc sine is computed, in the interval [-1,+1].
 * @returns Principal arc sine of x, in the interval [-pi/2,+pi/2] radians.
 */
template <dimensionless_unit ScalarUnit>
constexpr radian_t asin(ScalarUnit x) noexcept {
  return gcem::asin(x.value()) * radian;
}

/**
 * @ingroup UnitMath
 * @brief Compute arc tangent
 * @details Returns the principal value of the arc tangent of x, expressed in
 *          radians. Notice that because of the sign ambiguity, the function
 *          cannot determine with certainty in which quadrant the angle falls
 *          only by its tangent value. See atan2 for an alternative that takes a
 *          fractional argument instead.
 * @tparam AngleUnit  any angle `quantity` type.
 * @param[in] x Value whose arc tangent is computed, in the interval [-1,+1].
 * @returns Principal arc tangent of x, in the interval [-pi/2,+pi/2] radians.
 */
template <dimensionless_unit ScalarUnit>
constexpr radian_t atan(ScalarUnit x) noexcept {
  return gcem::atan(x.value()) * radian;
}

/**
 * @ingroup UnitMath
 * @brief Compute arc tangent with two parameters
 * @details To compute the value, the function takes into account the sign of
 *          both arguments in order to determine the quadrant.
 * @param[in] y y-component of the triangle expressed.
 * @param[in] x x-component of the triangle expressed.
 * @returns Returns the principal value of the arc tangent of <i>y/x</i>,
 *          expressed in radians.
 */
template <UnitT X, UnitT Y>
  requires requires(Y y, X x) {
    { y / x } -> dimensionless_unit;
  }
constexpr radian_t atan2(Y y, X x) noexcept {
  return gcem::atan2(X{y}.value(), x.value()) * radian;
}

//----------------------------------
// HYPERBOLIC TRIG FUNCTIONS
//----------------------------------

/**
 * @ingroup UnitMath
 * @brief Compute hyperbolic cosine
 * @details The input value can be in any unit of angle, including radians or
 *          degrees.
 * @tparam AngleUnit any angle `quantity` type.
 * @param[in] angle angle to compute the hyperbolic cosine of
 * @returns Returns the hyperbolic cosine of <i>angle</i>
 */
template <angle_unit AngleUnit>
constexpr scalar_t cosh(AngleUnit angle) noexcept {
  return gcem::cosh(radian_t{angle}.value()) * scalar;
}

/**
 * @ingroup UnitMath
 * @brief Compute hyperbolic sine
 * @details The input value can be in any unit of angle, including radians or
 *          degrees.
 * @tparam AngleUnit any angle `quantity` type.
 * @param[in] angle angle to compute the hyperbolic sine of
 * @returns Returns the hyperbolic sine of <i>angle</i>
 */
template <angle_unit AngleUnit>
constexpr scalar_t sinh(AngleUnit angle) noexcept {
  return gcem::sinh(radian_t{angle}.value()) * scalar;
}

/**
 * @ingroup UnitMath
 * @brief Compute hyperbolic tangent
 * @details The input value can be in any unit of angle, including radians or
 *          degrees.
 * @tparam AngleUnit any angle `quantity` type.
 * @param[in] angle angle to compute the hyperbolic tangent of
 * @returns Returns the hyperbolic tangent of <i>angle</i>
 */
template <angle_unit AngleUnit>
constexpr scalar_t tanh(AngleUnit angle) noexcept {
  return gcem::tanh(radian_t{angle}.value()) * scalar;
}

/**
 * @ingroup UnitMath
 * @brief Compute arc hyperbolic cosine
 * @details Returns the nonnegative arc hyperbolic cosine of x, expressed in
 *          radians.
 * @param[in] x Value whose arc hyperbolic cosine is computed. If the argument
 *              is less than 1, a domain error occurs.
 * @returns Nonnegative arc hyperbolic cosine of x, in the interval
 *          [0,+INFINITY] radians.
 */
template <dimensionless_unit ScalarUnit>
constexpr radian_t acosh(ScalarUnit x) noexcept {
  return gcem::acosh(x.value()) * radian;
}

/**
 * @ingroup UnitMath
 * @brief Compute arc hyperbolic sine
 * @details Returns the arc hyperbolic sine of x, expressed in radians.
 * @param[in] x Value whose arc hyperbolic sine is computed.
 * @returns Arc hyperbolic sine of x, in radians.
 */
template <dimensionless_unit ScalarUnit>
constexpr radian_t asinh(ScalarUnit x) noexcept {
  return gcem::asinh(x.value()) * radian;
}

/**
 * @ingroup UnitMath
 * @brief Compute arc hyperbolic tangent
 * @details Returns the arc hyperbolic tangent of x, expressed in radians.
 * @param[in] x Value whose arc hyperbolic tangent is computed, in the interval
 *              [-1,+1]. If the argument is out of this interval, a domain error
 *              occurs. For values of -1 and +1, a pole error may occur.
 * @returns units::radian_t
 */
template <dimensionless_unit ScalarUnit>
constexpr radian_t atanh(ScalarUnit x) noexcept {
  return gcem::atanh(x.value()) * radian;
}

//----------------------------------
// TRANSCENDENTAL FUNCTIONS
//----------------------------------

// it makes NO SENSE to put dimensioned units into a transcendental function,
// and if you think it does you are demonstrably wrong.
// https://en.wikipedia.org/wiki/Transcendental_function#Dimensional_analysis

/**
 * @ingroup UnitMath
 * @brief Compute exponential function
 * @details Returns the base-e exponential function of x, which is e raised to
 *          the power x: e^x.
 * @param[in] x scalar value of the exponent.
 * @returns Exponential value of x.
 *          If the magnitude of the result is too large to be represented by a
 *          value of the return type, the function returns HUGE_VAL (or
 *          HUGE_VALF or HUGE_VALL) with the proper sign, and an overflow range
 *          error occurs.
 */
template <dimensionless_unit ScalarUnit>
constexpr scalar_t exp(ScalarUnit x) noexcept {
  return gcem::exp(x.value()) * scalar;
}

/**
 * @ingroup UnitMath
 * @brief Compute natural logarithm
 * @details Returns the natural logarithm of x.
 * @param[in] x scalar value whose logarithm is calculated. If the argument is
 *              negative, a domain error occurs.
 * @sa log10 for more common base-10 logarithms
 * @returns Natural logarithm of x.
 */
template <dimensionless_unit ScalarUnit>
constexpr scalar_t log(ScalarUnit x) noexcept {
  return gcem::log(x.value()) * scalar;
}

/**
 * @ingroup UnitMath
 * @brief Compute common logarithm
 * @details Returns the common (base-10) logarithm of x.
 * @param[in] x Value whose logarithm is calculated. If the argument is
 *              negative, a domain error occurs.
 * @returns Common logarithm of x.
 */
template <dimensionless_unit ScalarUnit>
constexpr scalar_t log10(ScalarUnit x) noexcept {
  return gcem::log10(x.value()) * scalar;
}

/**
 * @ingroup UnitMath
 * @brief Break into fractional and integral parts.
 * @details The integer part is stored in the object pointed by intpart, and the
 *          fractional part is returned by the function. Both parts have the
 *          same sign as x.
 * @param[in] x scalar value to break into parts.
 * @param[in] intpart Pointer to an object (of the same type as x) where the
 *                    integral part is stored with the same sign as x.
 * @returns The fractional part of x, with the same sign.
 */
template <dimensionless_unit ScalarUnit>
scalar_t modf(ScalarUnit x, ScalarUnit* intpart) noexcept {
  typename ScalarUnit::rep intp;
  scalar_t fracpart = std::modf(x.value(), &intp) * scalar;
  *intpart = intp;
  return fracpart;
}

/**
 * @ingroup UnitMath
 * @brief Compute binary exponential function
 * @details Returns the base-2 exponential function of x, which is 2 raised to
 *          the power x: 2^x.
 * param[in]  x  Value of the exponent.
 * @returns 2 raised to the power of x.
 */
template <dimensionless_unit ScalarUnit>
scalar_t exp2(ScalarUnit x) noexcept {
  return std::exp2(x.value()) * scalar;
}

/**
 * @ingroup UnitMath
 * @brief Compute exponential minus one
 * @details Returns e raised to the power x minus one: e^x-1. For small
 *          magnitude values of x, expm1 may be more accurate than exp(x)-1.
 * @param[in] x Value of the exponent.
 * @returns e raised to the power of x, minus one.
 */
template <dimensionless_unit ScalarUnit>
scalar_t expm1(ScalarUnit x) noexcept {
  return gcem::expm1(x.value()) * scalar;
}

/**
 * @ingroup UnitMath
 * @brief Compute logarithm plus one
 * @details Returns the natural logarithm of one plus x. For small magnitude
 *          values of x, logp1 may be more accurate than log(1+x).
 * @param[in] x Value whose logarithm is calculated. If the argument is less
 *              than -1, a domain error occurs.
 * @returns The natural logarithm of (1+x).
 */
template <dimensionless_unit ScalarUnit>
scalar_t log1p(ScalarUnit x) noexcept {
  return gcem::log1p(x.value()) * scalar;
}

/**
 * @ingroup UnitMath
 * @brief Compute binary logarithm
 * @details Returns the binary (base-2) logarithm of x.
 * @param[in] x Value whose logarithm is calculated. If the argument is
 *              negative, a domain error occurs.
 * @returns The binary logarithm of x: log2x.
 */
template <dimensionless_unit ScalarUnit>
scalar_t log2(ScalarUnit x) noexcept {
  return gcem::log2(x.value()) * scalar;
}

//----------------------------------
// POWER FUNCTIONS
//----------------------------------

/* pow is implemented earlier in the library since a lot of the unit definitions
 * depend on it */

/**
 * @ingroup UnitMath
 * @brief computes the square root of <i>value</i>
 * @param[in] value `unit_t` derived type to compute the square root of.
 * @returns new unit_t, whose units are the square root of value's.
 *          E.g. if values had units of `square_meter`, then the return type
 *          will have units of `meter`.
 * @note `sqrt` provides a _rational approximation_ of the square root of
 *       <i>value</i>. In some cases, _both_ the returned value _and_ conversion
 *       factor of the returned unit type may have errors no larger than
 *       `1e-10`.
 */
template <UnitT UnitType>
inline constexpr auto sqrt(const UnitType& value) noexcept {
  return gcem::sqrt(value.value()) * mp_units::sqrt(UnitType::unit);
}

/**
 * @ingroup UnitMath
 * @brief Computes the square root of the sum-of-squares of x and y.
 * @param[in] x unit_t type value
 * @param[in] y unit_t type value
 * @returns square root of the sum-of-squares of x and y in the same units as x.
 */
template <UnitT UnitTypeLhs, UnitT UnitTypeRhs>
  requires std::is_convertible_v<UnitTypeRhs, UnitTypeLhs>
inline constexpr auto hypot(const UnitTypeLhs& x, const UnitTypeRhs& y) {
  return gcem::hypot(x.value(), UnitTypeLhs{y}.value()) * UnitTypeLhs::unit;
}

//----------------------------------
// ROUNDING FUNCTIONS
//----------------------------------

/**
 * @ingroup UnitMath
 * @brief Round up value
 * @details Rounds x upward, returning the smallest integral value that is not
 *          less than x.
 * @param[in] x Unit value to round up.
 * @returns The smallest integral value that is not less than x.
 */
template <UnitT UnitType>
constexpr UnitType ceil(UnitType x) noexcept {
  return gcem::ceil(x.value()) * UnitType::unit;
}

/**
 * @ingroup UnitMath
 * @brief Round down value
 * @details Rounds x downward, returning the largest integral value that is not
 *          greater than x.
 * @param[in] x Unit value to round down.
 * @returns The value of x rounded downward.
 */
template <UnitT UnitType>
constexpr UnitType floor(UnitType x) {
  return gcem::floor(x.value()) * UnitType::unit;
}

/**
 * @ingroup UnitMath
 * @brief Compute remainder of division
 * @details Returns the floating-point remainder of numer/denom (rounded towards
 *          zero).
 * @param[in] numer Value of the quotient numerator.
 * @param[in] denom Value of the quotient denominator.
 * @returns The remainder of dividing the arguments.
 */
template <UnitT UnitTypeLhs, UnitT UnitTypeRhs>
constexpr UnitTypeLhs fmod(UnitTypeLhs numer, UnitTypeRhs denom) noexcept {
  return gcem::fmod(numer.value(), UnitTypeLhs{numer}.value()) *
         UnitTypeLhs::unit;
}

/**
 * @ingroup UnitMath
 * @brief Truncate value
 * @details Rounds x toward zero, returning the nearest integral value that is
 *          not larger in magnitude than x. Effectively rounds towards 0.
 * @param[in] x Value to truncate
 * @returns The nearest integral value that is not larger in magnitude than x.
 */
template <UnitT UnitType>
constexpr UnitType trunc(UnitType x) noexcept {
  return gcem::trunc(x.value()) * UnitType::unit;
}

/**
 * @ingroup UnitMath
 * @brief Round to nearest
 * @details Returns the integral value that is nearest to x, with halfway cases
 *          rounded away from zero.
 * @param[in] x value to round.
 * @returns The value of x rounded to the nearest integral.
 */
template <UnitT UnitType>
constexpr UnitType round(UnitType x) noexcept {
  return gcem::round(x.value()) * UnitType::unit;
}

//----------------------------------
// FLOATING POINT MANIPULATION
//----------------------------------

/**
 * @ingroup UnitMath
 * @brief Copy sign
 * @details Returns a value with the magnitude and dimension of x, and the sign
 *          of y. Values x and y do not have to be compatible units.
 * @param[in] x Value with the magnitude of the resulting value.
 * @param[in] y Value with the sign of the resulting value.
 * @returns value with the magnitude and dimension of x, and the sign of y.
 */
template <UnitT UnitTypeLhs, UnitT UnitTypeRhs>
constexpr UnitTypeLhs copysign(UnitTypeLhs x, UnitTypeRhs y) noexcept {
  return gcem::copysign(x.value(), y.value()) * UnitTypeLhs::unit;
}

/// Overload to copy the sign from a raw double
template <UnitT UnitTypeLhs>
constexpr UnitTypeLhs copysign(UnitTypeLhs x, double y) noexcept {
  return gcem::copysign(x.value(), y) * UnitTypeLhs::unit;
}

//----------------------------------
// MIN / MAX / DIFFERENCE
//----------------------------------

/**
 * @ingroup UnitMath
 * @brief Positive difference
 * @details The function returns x-y if x>y, and zero otherwise, in the same
 *          units as x. Values x and y do not have to be the same type of units,
 *          but they do have to be compatible.
 * @param[in] x Values whose difference is calculated.
 * @param[in] y Values whose difference is calculated.
 * @returns The positive difference between x and y.
 */
template <UnitT UnitTypeLhs, UnitT UnitTypeRhs>
  requires std::is_convertible_v<UnitTypeRhs, UnitTypeLhs>
UnitTypeLhs fdim(UnitTypeLhs x, UnitTypeRhs y) noexcept {
  return std::fdim(x.value(), UnitTypeRhs{y}.value()) * UnitTypeLhs::unit;
}

/**
 * @ingroup UnitMath
 * @brief Maximum value
 * @details Returns the larger of its arguments: either x or y, in the same
 *          units as x. Values x and y do not have to be the same type of units,
 *          but they do have to be compatible.
 * @param[in] x Values among which the function selects a maximum.
 * @param[in] y Values among which the function selects a maximum.
 * @returns The maximum numeric value of its arguments.
 */
template <UnitT UnitTypeLhs, UnitT UnitTypeRhs>
  requires std::is_convertible_v<UnitTypeRhs, UnitTypeLhs>
constexpr UnitTypeLhs fmax(UnitTypeLhs x, UnitTypeRhs y) noexcept {
  return gcem::max(x.value(), UnitTypeLhs{y}.value()) * UnitTypeLhs::unit;
}

/**
 * @ingroup UnitMath
 * @brief Minimum value
 * @details Returns the smaller of its arguments: either x or y, in the same
 *          units as x. If one of the arguments in a NaN, the other is returned.
 *          Values x and y do not have to be the same type of units, but they do
 *          have to be compatible.
 * @param[in] x Values among which the function selects a minimum.
 * @param[in] y Values among which the function selects a minimum.
 * @returns The minimum numeric value of its arguments.
 */
template <UnitT UnitTypeLhs, UnitT UnitTypeRhs>
  requires std::is_convertible_v<UnitTypeRhs, UnitTypeLhs>
constexpr UnitTypeLhs fmin(UnitTypeLhs x, const UnitTypeRhs y) noexcept {
  return gcem::min(x.value(), UnitTypeLhs{y}.value()) * UnitTypeLhs::unit;
}

//----------------------------------
// OTHER FUNCTIONS
//----------------------------------

/**
 * @ingroup UnitMath
 * @brief Compute absolute value
 * @details Returns the absolute value of x, i.e. |x|.
 * @param[in] x Value whose absolute value is returned.
 * @returns The absolute value of x.
 */
template <UnitT UnitType>
constexpr UnitType fabs(const UnitType x) noexcept {
  return gcem::abs(x.value()) * UnitType::unit;
}

/**
 * @ingroup UnitMath
 * @brief Compute absolute value
 * @details Returns the absolute value of x, i.e. |x|.
 * @param[in] x Value whose absolute value is returned.
 * @returns The absolute value of x.
 */
template <UnitT UnitType>
constexpr UnitType abs(const UnitType x) noexcept {
  return gcem::abs(x()) * UnitType::unit;
}

/**
 * @ingroup UnitMath
 * @brief Multiply-add
 * @details Returns x*y+z. The function computes the result without losing
 *          precision in any intermediate result. The resulting unit type is a
 *          compound unit of x* y.
 * @param[in] x Values to be multiplied.
 * @param[in] y Values to be multiplied.
 * @param[in] z Value to be added.
 * @returns The result of x*y+z
 */
template <UnitT UnitTypeLhs, UnitT UnitMultiply, UnitT UnitAdd>
  requires requires(UnitTypeLhs x, UnitMultiply y, UnitAdd z) {
    requires std::is_convertible_v<UnitAdd, decltype(x * y)>;
  }
auto fma(const UnitTypeLhs x, const UnitMultiply y,
         const UnitAdd z) noexcept -> decltype(x * y) {
  using resultType = decltype(x * y);
  return std::fma(x.value(), y.value(), resultType{z}.value()) *
         resultType::unit;
}

}  // namespace units::math
