// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <type_traits>

#include "frc/units.h"
#include "wpimath/MathShared.h"

namespace frc {

/**
 * A trapezoid-shaped velocity profile.
 *
 * While this class can be used for a profiled movement from start to finish,
 * the intended usage is to filter a reference's dynamics based on trapezoidal
 * velocity constraints. To compute the reference obeying this constraint, do
 * the following.
 *
 * Initialization:
 * @code{.cpp}
 * TrapezoidProfile::Constraints constraints{kMaxV, kMaxA};
 * double previousProfiledReference = initialReference;
 * TrapezoidProfile profile{constraints};
 * @endcode
 *
 * Run on update:
 * @code{.cpp}
 * previousProfiledReference = profile.Calculate(timeSincePreviousUpdate,
 *                                               previousProfiledReference,
 *                                               unprofiledReference);
 * @endcode
 *
 * where `unprofiledReference` is free to change between calls. Note that when
 * the unprofiled reference is within the constraints, `Calculate()` returns the
 * unprofiled reference unchanged.
 *
 * Otherwise, a timer can be started to provide monotonic values for
 * `Calculate()` and to determine when the profile has completed via
 * `IsFinished()`.
 */
template <mp::Unit auto Distance>
class TrapezoidProfile {
 public:
  using Distance_t = mp::quantity<Distance>;
  inline static constexpr auto Velocity = Distance / mp::s;
  using Velocity_t = mp::quantity<Velocity>;
  inline static constexpr auto Acceleration = Velocity / mp::s;
  using Acceleration_t = mp::quantity<Acceleration>;

  /**
   * Profile constraints.
   */
  class Constraints {
   public:
    /// Maximum velocity.
    Velocity_t maxVelocity = 0.0 * Velocity;

    /// Maximum acceleration.
    Acceleration_t maxAcceleration = 0.0 * Acceleration;

    /**
     * Default constructor.
     */
    constexpr Constraints() {
      if (!std::is_constant_evaluated()) {
        wpi::math::MathSharedStore::ReportUsage("TrapezoidProfile", "");
      }
    }

    /**
     * Constructs constraints for a Trapezoid Profile.
     *
     * @param maxVelocity Maximum velocity, must be non-negative.
     * @param maxAcceleration Maximum acceleration, must be non-negative.
     */
    constexpr Constraints(Velocity_t maxVelocity,
                          Acceleration_t maxAcceleration)
        : maxVelocity{maxVelocity}, maxAcceleration{maxAcceleration} {
      if (!std::is_constant_evaluated()) {
        wpi::math::MathSharedStore::ReportUsage("TrapezoidProfile", "");
      }

      if (maxVelocity < 0.0 * Velocity ||
          maxAcceleration < 0.0 * Acceleration) {
        throw std::domain_error("Constraints must be non-negative");
      }
    }
  };

  /**
   * Profile state.
   */
  class State {
   public:
    /// The position at this state.
    Distance_t position = 0.0 * Distance;

    /// The velocity at this state.
    Velocity_t velocity = 0.0 * Velocity;

    constexpr bool operator==(const State&) const = default;
  };

  /**
   * Constructs a TrapezoidProfile.
   *
   * @param constraints The constraints on the profile, like maximum velocity.
   */
  constexpr TrapezoidProfile(Constraints constraints)  // NOLINT
      : m_constraints(constraints) {}

  constexpr TrapezoidProfile(const TrapezoidProfile&) = default;
  constexpr TrapezoidProfile& operator=(const TrapezoidProfile&) = default;
  constexpr TrapezoidProfile(TrapezoidProfile&&) = default;
  constexpr TrapezoidProfile& operator=(TrapezoidProfile&&) = default;

  /**
   * Calculates the position and velocity for the profile at a time t where the
   * current state is at time t = 0.
   *
   * @param t How long to advance from the current state toward the desired
   *     state.
   * @param current The current state.
   * @param goal The desired state when the profile is complete.
   * @return The position and velocity of the profile at time t.
   */
  constexpr State Calculate(mp::quantity<mp::s> t, State current, State goal) {
    m_direction = ShouldFlipAcceleration(current, goal) ? -1 : 1;
    m_current = Direct(current);
    goal = Direct(goal);
    if (mp::abs(m_current.velocity) > m_constraints.maxVelocity) {
      m_current.velocity =
          mp::copysign(m_constraints.maxVelocity, m_current.velocity);
    }

    // Deal with a possibly truncated motion profile (with nonzero initial or
    // final velocity) by calculating the parameters as if the profile began and
    // ended at zero velocity
    mp::quantity<mp::s> cutoffBegin =
        m_current.velocity / m_constraints.maxAcceleration;
    Distance_t cutoffDistBegin =
        cutoffBegin * cutoffBegin * m_constraints.maxAcceleration / 2.0;

    mp::quantity<mp::s> cutoffEnd =
        goal.velocity / m_constraints.maxAcceleration;
    Distance_t cutoffDistEnd =
        cutoffEnd * cutoffEnd * m_constraints.maxAcceleration / 2.0;

    // Now we can calculate the parameters as if it was a full trapezoid instead
    // of a truncated one

    Distance_t fullTrapezoidDist =
        cutoffDistBegin + (goal.position - m_current.position) + cutoffDistEnd;
    mp::quantity<mp::s> accelerationTime =
        m_constraints.maxVelocity / m_constraints.maxAcceleration;

    Distance_t fullSpeedDist =
        fullTrapezoidDist -
        accelerationTime * accelerationTime * m_constraints.maxAcceleration;

    // Handle the case where the profile never reaches full speed
    if (fullSpeedDist < 0.0 * Distance) {
      accelerationTime =
          mp::sqrt(fullTrapezoidDist / m_constraints.maxAcceleration);
      fullSpeedDist = 0.0 * Distance;
    }

    m_endAccel = accelerationTime - cutoffBegin;
    m_endFullSpeed = m_endAccel + fullSpeedDist / m_constraints.maxVelocity;
    m_endDecel = m_endFullSpeed + accelerationTime - cutoffEnd;
    State result = m_current;

    if (t < m_endAccel) {
      result.velocity += t * m_constraints.maxAcceleration;
      result.position +=
          (m_current.velocity + t * m_constraints.maxAcceleration / 2.0) * t;
    } else if (t < m_endFullSpeed) {
      result.velocity = m_constraints.maxVelocity;
      result.position += (m_current.velocity +
                          m_endAccel * m_constraints.maxAcceleration / 2.0) *
                             m_endAccel +
                         m_constraints.maxVelocity * (t - m_endAccel);
    } else if (t <= m_endDecel) {
      result.velocity =
          goal.velocity + (m_endDecel - t) * m_constraints.maxAcceleration;
      mp::quantity<mp::s> timeLeft = m_endDecel - t;
      result.position =
          goal.position -
          (goal.velocity + timeLeft * m_constraints.maxAcceleration / 2.0) *
              timeLeft;
    } else {
      result = goal;
    }

    return Direct(result);
  }

  /**
   * Returns the time left until a target distance in the profile is reached.
   *
   * @param target The target distance.
   * @return The time left until a target distance in the profile is reached, or
   * zero if no goal was set.
   */
  constexpr mp::quantity<mp::s> TimeLeftUntil(Distance_t target) const {
    Distance_t position = m_current.position * m_direction;
    Velocity_t velocity = m_current.velocity * m_direction;

    mp::quantity<mp::s> endAccel = m_endAccel * m_direction;
    mp::quantity<mp::s> endFullSpeed = m_endFullSpeed * m_direction - endAccel;

    if (target < position) {
      endAccel *= -1.0;
      endFullSpeed *= -1.0;
      velocity *= -1.0;
    }

    endAccel = std::max(endAccel, 0.0 * mp::s);
    endFullSpeed = std::max(endFullSpeed, 0.0 * mp::s);

    const Acceleration_t acceleration = m_constraints.maxAcceleration;
    const Acceleration_t deceleration = -m_constraints.maxAcceleration;

    Distance_t distToTarget = mp::abs(target - position);

    if (distToTarget < 1e-6 * Distance) {
      return 0.0 * mp::s;
    }

    Distance_t accelDist =
        velocity * endAccel + 0.5 * acceleration * endAccel * endAccel;

    Velocity_t decelVelocity;
    if (endAccel > 0.0 * mp::s) {
      decelVelocity =
          mp::sqrt(mp::abs(velocity * velocity + 2 * acceleration * accelDist));
    } else {
      decelVelocity = velocity;
    }

    Distance_t fullSpeedDist = m_constraints.maxVelocity * endFullSpeed;
    Distance_t decelDist;

    if (accelDist > distToTarget) {
      accelDist = distToTarget;
      fullSpeedDist = 0.0 * Distance;
      decelDist = 0.0 * Distance;
    } else if (accelDist + fullSpeedDist > distToTarget) {
      fullSpeedDist = distToTarget - accelDist;
      decelDist = 0.0 * Distance;
    } else {
      decelDist = distToTarget - fullSpeedDist - accelDist;
    }

    mp::quantity<mp::s> accelTime =
        (-velocity + mp::sqrt(mp::abs(velocity * velocity +
                                      2 * acceleration * accelDist))) /
        acceleration;

    mp::quantity<mp::s> decelTime =
        (-decelVelocity + mp::sqrt(mp::abs(decelVelocity * decelVelocity +
                                           2 * deceleration * decelDist))) /
        deceleration;

    mp::quantity<mp::s> fullSpeedTime =
        fullSpeedDist / m_constraints.maxVelocity;

    return accelTime + fullSpeedTime + decelTime;
  }

  /**
   * Returns the total time the profile takes to reach the goal.
   *
   * @return The total time the profile takes to reach the goal, or zero if no
   * goal was set.
   */
  constexpr mp::quantity<mp::s> TotalTime() const { return m_endDecel; }

  /**
   * Returns true if the profile has reached the goal.
   *
   * The profile has reached the goal if the time since the profile started has
   * exceeded the profile's total time.
   *
   * @param t The time since the beginning of the profile.
   * @return True if the profile has reached the goal.
   */
  constexpr bool IsFinished(mp::quantity<mp::s> t) const {
    return t >= TotalTime();
  }

 private:
  /**
   * Returns true if the profile inverted.
   *
   * The profile is inverted if goal position is less than the initial position.
   *
   * @param initial The initial state (usually the current state).
   * @param goal The desired state when the profile is complete.
   */
  static constexpr bool ShouldFlipAcceleration(const State& initial,
                                               const State& goal) {
    return initial.position > goal.position;
  }

  // Flip the sign of the velocity and position if the profile is inverted
  constexpr State Direct(const State& in) const {
    State result = in;
    result.position *= m_direction;
    result.velocity *= m_direction;
    return result;
  }

  // The direction of the profile, either 1 for forwards or -1 for inverted
  int m_direction = 1;

  Constraints m_constraints;
  State m_current;

  mp::quantity<mp::s> m_endAccel = 0.0 * mp::s;
  mp::quantity<mp::s> m_endFullSpeed = 0.0 * mp::s;
  mp::quantity<mp::s> m_endDecel = 0.0 * mp::s;
};

}  // namespace frc
