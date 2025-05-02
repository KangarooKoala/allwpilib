// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <gcem.hpp>
#include <wpi/MathExtras.h>

#include "frc/units.h"
#include "wpimath/MathShared.h"

namespace frc {

/**
 * A helper class that computes feedforward voltages for a simple
 * permanent-magnet DC motor.
 */
template <mp::Unit auto Distance>
  requires mp::UnitOf<Distance, mp::length> || mp::UnitOf<Distance, mp::angle>
class SimpleMotorFeedforward {
 public:
  inline constexpr auto Velocity = Distance / mp::s;
  inline constexpr auto Acceleration = Velocity / mp::s;
  inline constexpr auto kv_unit = mp::V / Velocity;
  inline constexpr auto ka_unit = mp::V / Acceleration;

  /**
   * Creates a new SimpleMotorFeedforward with the specified gains.
   *
   * @param kS The static gain, in volts.
   * @param kV The velocity gain, in volt seconds per distance.
   * @param kA The acceleration gain, in volt seconds² per distance.
   * @param dt The period in seconds.
   * @throws IllegalArgumentException for kv &lt; zero.
   * @throws IllegalArgumentException for ka &lt; zero.
   * @throws IllegalArgumentException for period &le; zero.
   */
  constexpr SimpleMotorFeedforward(mp::quantity<mp::V> kS,
                                   mp::quantity<kv_unit> kV,
                                   mp::quantity<ka_unit> kA = 0.0 * ka_unit,
                                   mp::quantity<mp::s> dt = 20.0 * mp::ms)
      : kS(kS), kV(kV), kA(kA), m_dt(dt) {
    if (mp::value(kV) < 0) {
      wpi::math::MathSharedStore::ReportError(
          "kV must be a non-negative number, got {}!", mp::value(kV));
      this->kV = 0.0 * kv_unit;
      wpi::math::MathSharedStore::ReportWarning("kV defaulted to 0.");
    }
    if (mp::value(kA) < 0) {
      wpi::math::MathSharedStore::ReportError(
          "kA must be a non-negative number, got {}!", mp::value(kA));
      this->kA = 0.0 * ka_unit;
      wpi::math::MathSharedStore::ReportWarning("kA defaulted to 0.");
    }
    if (dt <= 0.0 * mp::ms) {
      wpi::math::MathSharedStore::ReportError(
          "period must be a positive number, got {}!", mp::value(dt));
      this->m_dt = 20.0 * mp::ms;
      wpi::math::MathSharedStore::ReportWarning("period defaulted to 20 ms.");
    }
  }

  /**
   * Calculates the feedforward from the gains and velocity setpoint assuming
   * discrete control. Use this method when the velocity setpoint does not
   * change.
   *
   * @param velocity The velocity setpoint.
   * @return The computed feedforward, in volts.
   */
  constexpr mp::quantity<mp::V> Calculate(
      mp::quantity<Velocity> velocity) const {
    return Calculate(velocity, velocity);
  }

  /**
   * Calculates the feedforward from the gains and setpoints assuming discrete
   * control.
   *
   * <p>Note this method is inaccurate when the velocity crosses 0.
   *
   * @param currentVelocity The current velocity setpoint.
   * @param nextVelocity    The next velocity setpoint.
   * @return The computed feedforward, in volts.
   */
  constexpr mp::quantity<mp::V> Calculate(
      mp::quantity<Velocity> currentVelocity,
      mp::quantity<Velocity> nextVelocity) const {
    // See wpimath/algorithms.md#Simple_motor_feedforward for derivation
    if (kA < 1e-9 * decltype(kA)) {
      return kS * wpi::sgn(nextVelocity) + kV * nextVelocity;
    } else {
      double A = mp::value(-kV) / mp::value(kA);
      double B = 1.0 / mp::value(kA);
      double A_d = gcem::exp(A * mp::value(m_dt));
      double B_d = A > -1e-9 ? B * mp::value(m_dt) : 1.0 / A * (A_d - 1.0) * B;
      return kS * wpi::sgn(currentVelocity) +
             1.0 / B_d *
                 (mp::value(nextVelocity) - A_d * mp::value(currentVelocity)) *
                 mp::V;
    }
  }

  // Rearranging the main equation from the calculate() method yields the
  // formulas for the methods below:

  /**
   * Calculates the maximum achievable velocity given a maximum voltage supply
   * and an acceleration.  Useful for ensuring that velocity and
   * acceleration constraints for a trapezoidal profile are simultaneously
   * achievable - enter the acceleration constraint, and this will give you
   * a simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the motor.
   * @param acceleration The acceleration of the motor.
   * @return The maximum possible velocity at the given acceleration.
   */
  constexpr mp::quantity<Velocity> MaxAchievableVelocity(
      mp::quantity<mp::V> maxVoltage,
      mp::quantity<Acceleration> acceleration) const {
    // Assume max velocity is positive
    return (maxVoltage - kS - kA * acceleration) / kV;
  }

  /**
   * Calculates the minimum achievable velocity given a maximum voltage supply
   * and an acceleration.  Useful for ensuring that velocity and
   * acceleration constraints for a trapezoidal profile are simultaneously
   * achievable - enter the acceleration constraint, and this will give you
   * a simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the motor.
   * @param acceleration The acceleration of the motor.
   * @return The minimum possible velocity at the given acceleration.
   */
  constexpr mp::quantity<Velocity> MinAchievableVelocity(
      mp::quantity<mp::V> maxVoltage,
      mp::quantity<Acceleration> acceleration) const {
    // Assume min velocity is positive, ks flips sign
    return (-maxVoltage + kS - kA * acceleration) / kV;
  }

  /**
   * Calculates the maximum achievable acceleration given a maximum voltage
   * supply and a velocity. Useful for ensuring that velocity and
   * acceleration constraints for a trapezoidal profile are simultaneously
   * achievable - enter the velocity constraint, and this will give you
   * a simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the motor.
   * @param velocity The velocity of the motor.
   * @return The maximum possible acceleration at the given velocity.
   */
  constexpr mp::quantity<Acceleration> MaxAchievableAcceleration(
      mp::quantity<mp::V> maxVoltage, mp::quantity<Velocity> velocity) const {
    return (maxVoltage - kS * wpi::sgn(velocity) - kV * velocity) / kA;
  }

  /**
   * Calculates the minimum achievable acceleration given a maximum voltage
   * supply and a velocity. Useful for ensuring that velocity and
   * acceleration constraints for a trapezoidal profile are simultaneously
   * achievable - enter the velocity constraint, and this will give you
   * a simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the motor.
   * @param velocity The velocity of the motor.
   * @return The minimum possible acceleration at the given velocity.
   */
  constexpr mp::quantity<Acceleration> MinAchievableAcceleration(
      mp::quantity<mp::V> maxVoltage, mp::quantity<Velocity> velocity) const {
    return MaxAchievableAcceleration(-maxVoltage, velocity);
  }

  /**
   * Sets the static gain.
   *
   * @param kS The static gain.
   */
  constexpr void SetKs(mp::quantity<mp::V> kS) { this->kS = kS; }

  /**
   * Sets the velocity gain.
   *
   * @param kV The velocity gain.
   */
  constexpr void SetKv(mp::quantity<kv_unit> kV) { this->kV = kV; }

  /**
   * Sets the acceleration gain.
   *
   * @param kA The acceleration gain.
   */
  constexpr void SetKa(mp::quantity<ka_unit> kA) { this->kA = kA; }

  /**
   * Returns the static gain.
   *
   * @return The static gain.
   */
  constexpr mp::quantity<mp::V> GetKs() const { return kS; }

  /**
   * Returns the velocity gain.
   *
   * @return The velocity gain.
   */
  constexpr mp::quantity<kv_unit> GetKv() const { return kV; }

  /**
   * Returns the acceleration gain.
   *
   * @return The acceleration gain.
   */
  constexpr mp::quantity<ka_unit> GetKa() const { return kA; }

  /**
   * Returns the period.
   *
   * @return The period.
   */
  constexpr mp::quantity<mp::s> GetDt() const { return m_dt; }

 private:
  /** The static gain. */
  mp::quantity<mp::V> kS;

  /** The velocity gain. */
  mp::quantity<kv_unit> kV;

  /** The acceleration gain. */
  mp::quantity<ka_unit> kA;

  /** The period. */
  mp::quantity<mp::s> m_dt;
};

}  // namespace frc
