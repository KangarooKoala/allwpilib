// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstdlib>

#include <wpi/MathExtras.h>
#include <wpi/SymbolExports.h>

#include "frc/units.h"
#include "wpimath/MathShared.h"

namespace frc {
/**
 * A helper class that computes feedforward outputs for a simple arm (modeled as
 * a motor acting against the force of gravity on a beam suspended at an angle).
 */
class WPILIB_DLLEXPORT ArmFeedforward {
 public:
  inline static constexpr auto Angle = mp::rad;
  inline static constexpr auto Velocity = Angle / mp::s;
  inline static constexpr auto Acceleration = Velocity / mp::s;
  inline static constexpr auto kv_unit = mp::V / Velocity;
  inline static constexpr auto ka_unit = mp::V / Acceleration;

  /**
   * Creates a new ArmFeedforward with the specified gains.
   *
   * @param kS The static gain, in volts.
   * @param kG The gravity gain, in volts.
   * @param kV The velocity gain, in volt seconds per radian.
   * @param kA The acceleration gain, in volt seconds² per radian.
   * @param dt The period in seconds.
   * @throws IllegalArgumentException for kv &lt; zero.
   * @throws IllegalArgumentException for ka &lt; zero.
   * @throws IllegalArgumentException for period &le; zero.
   */
  constexpr ArmFeedforward(mp::quantity<mp::V> kS, mp::quantity<mp::V> kG,
                           mp::quantity<kv_unit> kV,
                           mp::quantity<ka_unit> kA = 0.0 * ka_unit,
                           mp::quantity<mp::s> dt = 20.0 * mp::ms)
      : kS(kS), kG(kG), kV(kV), kA(kA), m_dt(dt) {
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
      wpi::math::MathSharedStore::ReportWarning("kA defaulted to 0;");
    }
    if (dt <= 0.0 * mp::ms) {
      wpi::math::MathSharedStore::ReportError(
          "period must be a positive number, got {}!", mp::value(dt));
      this->m_dt = 20.0 * mp::ms;
      wpi::math::MathSharedStore::ReportWarning("period defaulted to 20 ms.");
    }
  }

  /**
   * Calculates the feedforward from the gains and setpoints assuming continuous
   * control.
   *
   * @param angle        The angle setpoint, in radians. This angle should be
   *                     measured from the horizontal (i.e. if the provided
   *                     angle is 0, the arm should be parallel to the floor).
   *                     If your encoder does not follow this convention, an
   *                     offset should be added.
   * @param velocity     The velocity setpoint.
   * @param acceleration The acceleration setpoint.
   * @return The computed feedforward, in volts.
   */
  [[deprecated("Use the current/next velocity overload instead.")]]
  constexpr mp::quantity<mp::V> Calculate(
      mp::quantity<Angle> angle, mp::quantity<Velocity> velocity,
      mp::quantity<Acceleration> acceleration) const {
    return kS * wpi::sgn(velocity) + kG * mp::cos(angle) + kV * velocity +
           kA * acceleration;
  }

  /**
   * Calculates the feedforward from the gains and setpoints assuming continuous
   * control.
   *
   * @param currentAngle The current angle in radians. This angle should be
   *   measured from the horizontal (i.e. if the provided angle is 0, the arm
   *   should be parallel to the floor). If your encoder does not follow this
   *   convention, an offset should be added.
   * @param currentVelocity The current velocity setpoint.
   * @param nextVelocity The next velocity setpoint.
   * @param dt Time between velocity setpoints in seconds.
   * @return The computed feedforward in volts.
   */
  [[deprecated("Use the current/next velocity overload instead.")]]
  mp::quantity<mp::V> Calculate(mp::quantity<Angle> currentAngle,
                                mp::quantity<Velocity> currentVelocity,
                                mp::quantity<Velocity> nextVelocity,
                                mp::quantity<mp::s> dt) const {
    return Calculate(currentAngle, currentVelocity, nextVelocity);
  }

  /**
   * Calculates the feedforward from the gains and setpoint assuming discrete
   * control. Use this method when the velocity does not change.
   *
   * @param currentAngle The current angle. This angle should be measured from
   * the horizontal (i.e. if the provided angle is 0, the arm should be parallel
   * to the floor). If your encoder does not follow this convention, an offset
   * should be added.
   * @param currentVelocity The current velocity.
   * @return The computed feedforward in volts.
   */
  constexpr mp::quantity<mp::V> Calculate(
      mp::quantity<Angle> currentAngle,
      mp::quantity<Velocity> currentVelocity) const {
    return kS * wpi::sgn(currentVelocity) + kG * mp::cos(currentAngle) +
           kV * currentVelocity;
  }

  /**
   * Calculates the feedforward from the gains and setpoints assuming discrete
   * control.
   *
   * @param currentAngle The current angle. This angle should be measured from
   * the horizontal (i.e. if the provided angle is 0, the arm should be parallel
   * to the floor). If your encoder does not follow this convention, an offset
   * should be added.
   * @param currentVelocity The current velocity.
   * @param nextVelocity    The next velocity.
   * @return The computed feedforward in volts.
   */
  mp::quantity<mp::V> Calculate(mp::quantity<Angle> currentAngle,
                                mp::quantity<Velocity> currentVelocity,
                                mp::quantity<Velocity> nextVelocity) const;

  // Rearranging the main equation from the calculate() method yields the
  // formulas for the methods below:

  /**
   * Calculates the maximum achievable velocity given a maximum voltage supply,
   * a position, and an acceleration.  Useful for ensuring that velocity and
   * acceleration constraints for a trapezoidal profile are simultaneously
   * achievable - enter the acceleration constraint, and this will give you
   * a simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage   The maximum voltage that can be supplied to the arm.
   * @param angle        The angle of the arm. This angle should be measured
   *                     from the horizontal (i.e. if the provided angle is 0,
   *                     the arm should be parallel to the floor). If your
   *                     encoder does not follow this convention, an offset
   *                     should be added.
   * @param acceleration The acceleration of the arm.
   * @return The maximum possible velocity at the given acceleration and angle.
   */
  constexpr mp::quantity<Velocity> MaxAchievableVelocity(
      mp::quantity<mp::V> maxVoltage, mp::quantity<Angle> angle,
      mp::quantity<Acceleration> acceleration) {
    // Assume max velocity is positive
    return (maxVoltage - kS - kG * mp::cos(angle) - kA * acceleration) / kV;
  }

  /**
   * Calculates the minimum achievable velocity given a maximum voltage supply,
   * a position, and an acceleration.  Useful for ensuring that velocity and
   * acceleration constraints for a trapezoidal profile are simultaneously
   * achievable - enter the acceleration constraint, and this will give you
   * a simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage   The maximum voltage that can be supplied to the arm.
   * @param angle        The angle of the arm. This angle should be measured
   *                     from the horizontal (i.e. if the provided angle is 0,
   *                     the arm should be parallel to the floor). If your
   *                     encoder does not follow this convention, an offset
   *                     should be added.
   * @param acceleration The acceleration of the arm.
   * @return The minimum possible velocity at the given acceleration and angle.
   */
  constexpr mp::quantity<Velocity> MinAchievableVelocity(
      mp::quantity<mp::V> maxVoltage, mp::quantity<Angle> angle,
      mp::quantity<Acceleration> acceleration) {
    // Assume min velocity is negative, ks flips sign
    return (-maxVoltage + kS - kG * mp::cos(angle) - kA * acceleration) / kV;
  }

  /**
   * Calculates the maximum achievable acceleration given a maximum voltage
   * supply, a position, and a velocity. Useful for ensuring that velocity and
   * acceleration constraints for a trapezoidal profile are simultaneously
   * achievable - enter the velocity constraint, and this will give you
   * a simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm.
   * @param angle      The angle of the arm. This angle should be measured
   *                   from the horizontal (i.e. if the provided angle is 0,
   *                   the arm should be parallel to the floor). If your
   *                   encoder does not follow this convention, an offset
   *                   should be added.
   * @param velocity   The velocity of the arm.
   * @return The maximum possible acceleration at the given velocity and angle.
   */
  constexpr mp::quantity<Acceleration> MaxAchievableAcceleration(
      mp::quantity<mp::V> maxVoltage, mp::quantity<Angle> angle,
      mp::quantity<Velocity> velocity) {
    return (maxVoltage - kS * wpi::sgn(velocity) - kG * mp::cos(angle) -
            kV * velocity) /
           kA;
  }

  /**
   * Calculates the minimum achievable acceleration given a maximum voltage
   * supply, a position, and a velocity. Useful for ensuring that velocity and
   * acceleration constraints for a trapezoidal profile are simultaneously
   * achievable - enter the velocity constraint, and this will give you
   * a simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm.
   * @param angle      The angle of the arm. This angle should be measured
   *                   from the horizontal (i.e. if the provided angle is 0,
   *                   the arm should be parallel to the floor). If your
   *                   encoder does not follow this convention, an offset
   *                   should be added.
   * @param velocity   The velocity of the arm.
   * @return The minimum possible acceleration at the given velocity and angle.
   */
  constexpr mp::quantity<Acceleration> MinAchievableAcceleration(
      mp::quantity<mp::V> maxVoltage, mp::quantity<Angle> angle,
      mp::quantity<Velocity> velocity) {
    return MaxAchievableAcceleration(-maxVoltage, angle, velocity);
  }

  /**
   * Sets the static gain.
   *
   * @param kS The static gain.
   */
  constexpr void SetKs(mp::quantity<mp::V> kS) { this->kS = kS; }

  /**
   * Sets the gravity gain.
   *
   * @param kG The gravity gain.
   */
  constexpr void SetKg(mp::quantity<mp::V> kG) { this->kG = kG; }

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
   * Returns the gravity gain.
   *
   * @return The gravity gain.
   */
  constexpr mp::quantity<mp::V> GetKg() const { return kG; }

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

 private:
  /// The static gain, in volts.
  mp::quantity<mp::V> kS;

  /// The gravity gain, in volts.
  mp::quantity<mp::V> kG;

  /// The velocity gain, in V/(rad/s)volt seconds per radian.
  mp::quantity<kv_unit> kV;

  /// The acceleration gain, in V/(rad/s²).
  mp::quantity<ka_unit> kA;

  /** The period. */
  mp::quantity<mp::s> m_dt;
};
}  // namespace frc

#include "frc/controller/proto/ArmFeedforwardProto.h"
#include "frc/controller/struct/ArmFeedforwardStruct.h"
