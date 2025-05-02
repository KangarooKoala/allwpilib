// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>

#include <wpi/timestamp.h>

#include "frc/units.h"
#include "wpimath/MathShared.h"

namespace frc {
/**
 * A class that limits the rate of change of an input value.  Useful for
 * implementing voltage, setpoint, and/or output ramps.  A slew-rate limit
 * is most appropriate when the quantity being controlled is a velocity or
 * a voltage; when controlling a position, consider using a TrapezoidProfile
 * instead.
 *
 * @see TrapezoidProfile
 */
template <mp::Unit auto Unit>
class SlewRateLimiter {
 public:
  using Unit_t = mp::quantity<Unit>;
  inline static constexpr auto Rate = Unit / mp::s;
  using Rate_t = mp::quantity<Rate>;

  /**
   * Creates a new SlewRateLimiter with the given positive and negative rate
   * limits and initial value.
   *
   * @param positiveRateLimit The rate-of-change limit in the positive
   *                          direction, in units per second. This is expected
   *                          to be positive.
   * @param negativeRateLimit The rate-of-change limit in the negative
   *                          direction, in units per second. This is expected
   *                          to be negative.
   * @param initialValue The initial value of the input.
   */
  SlewRateLimiter(Rate_t positiveRateLimit, Rate_t negativeRateLimit,
                  Unit_t initialValue = 0.0 * Unit)
      : m_positiveRateLimit{positiveRateLimit},
        m_negativeRateLimit{negativeRateLimit},
        m_prevVal{initialValue},
        m_prevTime{wpi::math::MathSharedStore::GetTimestamp()} {}

  /**
   * Creates a new SlewRateLimiter with the given positive rate limit and
   * negative rate limit of -rateLimit.
   *
   * @param rateLimit The rate-of-change limit.
   */
  explicit SlewRateLimiter(Rate_t rateLimit)
      : SlewRateLimiter(rateLimit, -rateLimit) {}

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew
   * rate.
   */
  Unit_t Calculate(Unit_t input) {
    mp::quantity<mp::s> currentTime =
        wpi::math::MathSharedStore::GetTimestamp();
    mp::quantity<mp::s> elapsedTime = currentTime - m_prevTime;
    m_prevVal +=
        std::clamp(input - m_prevVal, m_negativeRateLimit * elapsedTime,
                   m_positiveRateLimit * elapsedTime);
    m_prevTime = currentTime;
    return m_prevVal;
  }

  /**
   * Returns the value last calculated by the SlewRateLimiter.
   *
   * @return The last value.
   */
  Unit_t LastValue() const { return m_prevVal; }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit
   * when doing so.
   *
   * @param value The value to reset to.
   */
  void Reset(Unit_t value) {
    m_prevVal = value;
    m_prevTime = wpi::math::MathSharedStore::GetTimestamp();
  }

 private:
  Rate_t m_positiveRateLimit;
  Rate_t m_negativeRateLimit;
  Unit_t m_prevVal;
  mp::quantity<mp::s> m_prevTime;
};
}  // namespace frc
