// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/units.h"

namespace frc {

/**
 * A Exponential-shaped velocity profile.
 *
 * While this class can be used for a profiled movement from start to finish,
 * the intended usage is to filter a reference's dynamics based on
 * ExponentialProfile velocity constraints. To compute the reference obeying
 * this constraint, do the following.
 *
 * Initialization:
 * @code{.cpp}
 * ExponentialProfile::Constraints constraints{kMaxV, kV, kA};
 * State previousProfiledReference = {initialReference, 0.0 * mp::m / mp::s};
 * @endcode
 *
 * Run on update:
 * @code{.cpp}
 * previousProfiledReference = profile.Calculate(timeSincePreviousUpdate,
 * previousProfiledReference, unprofiledReference);
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
template <mp::Unit auto Distance, mp::Unit auto Input>
class ExponentialProfile {
 public:
  using Distance_t = mp::quantity<Distance>;
  inline static constexpr auto Velocity = Distance / mp::s;
  using Velocity_t = mp::quantity<Velocity>;
  inline static constexpr auto Acceleration = Velocity / mp::s;
  using Input_t = mp::quantity<Input>;
  using A_t = mp::quantity<mp::one / mp::s>;
  using B_t = mp::quantity<Acceleration / Input>;
  inline static constexpr auto KV = Input / Velocity;
  using kV_t = mp::quantity<KV>;
  inline static constexpr auto KA = Input / Acceleration;
  using kA_t = mp::quantity<KA>;

  /**
   * Profile timing.
   */
  class ProfileTiming {
   public:
    /// Profile inflection time.
    mp::quantity<mp::s> inflectionTime;

    /// Total profile time.
    mp::quantity<mp::s> totalTime;

    /**
     * Decides if the profile is finished by time t.
     *
     * @param t The time since the beginning of the profile.
     * @return if the profile is finished at time t.
     */
    constexpr bool IsFinished(const mp::quantity<mp::s>& t) const {
      return t >= totalTime;
    }
  };

  /**
   * Profile constraints.
   */
  class Constraints {
   public:
    /**
     * Constructs constraints for an ExponentialProfile.
     *
     * @param maxInput maximum unsigned input voltage
     * @param A The State-Space 1x1 system matrix.
     * @param B The State-Space 1x1 input matrix.
     */
    constexpr Constraints(Input_t maxInput, A_t A, B_t B)
        : maxInput{maxInput}, A{A}, B{B} {}

    /**
     * Constructs constraints for an ExponentialProfile from characteristics.
     *
     * @param maxInput maximum unsigned input voltage
     * @param kV The velocity gain.
     * @param kA The acceleration gain.
     */
    constexpr Constraints(Input_t maxInput, kV_t kV, kA_t kA)
        : maxInput{maxInput}, A{-kV / kA}, B{1 / kA} {}

    /**
     * Computes the max achievable velocity for an Exponential Profile.
     *
     * @return The steady-state velocity achieved by this profile.
     */
    constexpr Velocity_t MaxVelocity() const { return -maxInput * B / A; }

    /// Maximum unsigned input voltage.
    Input_t maxInput = 0.0 * Input;

    /// The State-Space 1x1 system matrix.
    A_t A = 0.0 / mp::s;

    /// The State-Space 1x1 input matrix.
    B_t B = 0.0 * Acceleration / Input;
  };

  /** Profile state. */
  class State {
   public:
    /// The position at this state.
    Distance_t position = 0.0 * Distance;

    /// The velocity at this state.
    Velocity_t velocity = 0.0 * Velocity;

    constexpr bool operator==(const State&) const = default;
  };

  /**
   * Constructs a ExponentialProfile.
   *
   * @param constraints The constraints on the profile, like maximum input.
   */
  constexpr explicit ExponentialProfile(Constraints constraints)
      : m_constraints(constraints) {}

  constexpr ExponentialProfile(const ExponentialProfile&) = default;
  constexpr ExponentialProfile& operator=(const ExponentialProfile&) = default;
  constexpr ExponentialProfile(ExponentialProfile&&) = default;
  constexpr ExponentialProfile& operator=(ExponentialProfile&&) = default;

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
  constexpr State Calculate(const mp::quantity<mp::s>& t, const State& current,
                            const State& goal) const {
    auto direction = ShouldFlipInput(current, goal) ? -1 : 1;
    auto u = direction * m_constraints.maxInput;

    auto inflectionPoint = CalculateInflectionPoint(current, goal, u);
    auto timing = CalculateProfileTiming(current, inflectionPoint, goal, u);

    if (t < 0.0 * mp::s) {
      return current;
    } else if (t < timing.inflectionTime) {
      return {ComputeDistanceFromTime(t, u, current),
              ComputeVelocityFromTime(t, u, current)};
    } else if (t < timing.totalTime) {
      return {ComputeDistanceFromTime(t - timing.totalTime, -u, goal),
              ComputeVelocityFromTime(t - timing.totalTime, -u, goal)};
    } else {
      return goal;
    }
  }

  /**
   * Calculates the point after which the fastest way to reach the goal state is
   * to apply input in the opposite direction.
   *
   * @param current The current state.
   * @param goal The desired state when the profile is complete.
   * @return The position and velocity of the profile at the inflection point.
   */
  constexpr State CalculateInflectionPoint(const State& current,
                                           const State& goal) const {
    auto direction = ShouldFlipInput(current, goal) ? -1 : 1;
    auto u = direction * m_constraints.maxInput;

    return CalculateInflectionPoint(current, goal, u);
  }

  /**
   * Calculates the time it will take for this profile to reach the goal state.
   *
   * @param current The current state.
   * @param goal The desired state when the profile is complete.
   * @return The total duration of this profile.
   */
  constexpr mp::quantity<mp::s> TimeLeftUntil(const State& current,
                                              const State& goal) const {
    auto timing = CalculateProfileTiming(current, goal);

    return timing.totalTime;
  }

  /**
   * Calculates the time it will take for this profile to reach the inflection
   * point, and the time it will take for this profile to reach the goal state.
   *
   * @param current The current state.
   * @param goal The desired state when the profile is complete.
   * @return The timing information for this profile.
   */
  constexpr ProfileTiming CalculateProfileTiming(const State& current,
                                                 const State& goal) const {
    auto direction = ShouldFlipInput(current, goal) ? -1 : 1;
    auto u = direction * m_constraints.maxInput;

    auto inflectionPoint = CalculateInflectionPoint(current, goal, u);
    return CalculateProfileTiming(current, inflectionPoint, goal, u);
  }

 private:
  /**
   * Calculates the point after which the fastest way to reach the goal state is
   * to apply input in the opposite direction.
   *
   * @param current The current state.
   * @param goal The desired state when the profile is complete.
   * @param input The signed input applied to this profile from the current
   *     state.
   * @return The position and velocity of the profile at the inflection point.
   */
  constexpr State CalculateInflectionPoint(const State& current,
                                           const State& goal,
                                           const Input_t& input) const {
    auto u = input;

    if (current == goal) {
      return current;
    }

    auto inflectionVelocity = SolveForInflectionVelocity(u, current, goal);
    auto inflectionPosition =
        ComputeDistanceFromVelocity(inflectionVelocity, -u, goal);

    return {inflectionPosition, inflectionVelocity};
  }

  /**
   * Calculates the time it will take for this profile to reach the inflection
   * point, and the time it will take for this profile to reach the goal state.
   *
   * @param current The current state.
   * @param inflectionPoint The inflection point of this profile.
   * @param goal The desired state when the profile is complete.
   * @param input The signed input applied to this profile from the current
   *     state.
   * @return The timing information for this profile.
   */
  constexpr ProfileTiming CalculateProfileTiming(const State& current,
                                                 const State& inflectionPoint,
                                                 const State& goal,
                                                 const Input_t& input) const {
    auto u = input;
    auto u_dir = mp::abs(u) / u;

    mp::quantity<mp::s> inflectionT_forward;

    // We need to handle 5 cases here:
    //
    // - Approaching -maxVelocity from below
    // - Approaching -maxVelocity from above
    // - Approaching maxVelocity from below
    // - Approaching maxVelocity from above
    // - At +-maxVelocity
    //
    // For cases 1 and 3, we want to subtract epsilon from the inflection point
    // velocity For cases 2 and 4, we want to add epsilon to the inflection
    // point velocity. For case 5, we have reached inflection point velocity.
    auto epsilon = 1e-9 * Velocity;
    if (mp::abs(u_dir * m_constraints.MaxVelocity() -
                inflectionPoint.velocity) < epsilon) {
      auto solvableV = inflectionPoint.velocity;
      mp::quantity<mp::s> t_to_solvable_v;
      Distance_t x_at_solvable_v;
      if (mp::abs(current.velocity - inflectionPoint.velocity) < epsilon) {
        t_to_solvable_v = 0.0 * mp::s;
        x_at_solvable_v = current.position;
      } else {
        if (mp::abs(current.velocity) > m_constraints.MaxVelocity()) {
          solvableV += u_dir * epsilon;
        } else {
          solvableV -= u_dir * epsilon;
        }

        t_to_solvable_v =
            ComputeTimeFromVelocity(solvableV, u, current.velocity);
        x_at_solvable_v = ComputeDistanceFromVelocity(solvableV, u, current);
      }

      inflectionT_forward =
          t_to_solvable_v + u_dir *
                                (inflectionPoint.position - x_at_solvable_v) /
                                m_constraints.MaxVelocity();
    } else {
      inflectionT_forward = ComputeTimeFromVelocity(inflectionPoint.velocity, u,
                                                    current.velocity);
    }

    auto inflectionT_backward =
        ComputeTimeFromVelocity(inflectionPoint.velocity, -u, goal.velocity);

    return {inflectionT_forward, inflectionT_forward - inflectionT_backward};
  }

  /**
   * Calculates the position reached after t seconds when applying an input from
   * the initial state.
   *
   * @param t The time since the initial state.
   * @param input The signed input applied to this profile from the initial
   *     state.
   * @param initial The initial state.
   * @return The distance travelled by this profile.
   */
  constexpr Distance_t ComputeDistanceFromTime(const mp::quantity<mp::s>& time,
                                               const Input_t& input,
                                               const State& initial) const {
    auto A = m_constraints.A;
    auto B = m_constraints.B;
    auto u = input;

    return initial.position + (-B * u * time + (initial.velocity + B * u / A) *
                                                   (mp::exp(A * time) - 1)) /
                                  A;
  }

  /**
   * Calculates the velocity reached after t seconds when applying an input from
   * the initial state.
   *
   * @param t The time since the initial state.
   * @param input The signed input applied to this profile from the initial
   *     state.
   * @param initial The initial state.
   * @return The distance travelled by this profile.
   */
  constexpr Velocity_t ComputeVelocityFromTime(const mp::quantity<mp::s>& time,
                                               const Input_t& input,
                                               const State& initial) const {
    auto A = m_constraints.A;
    auto B = m_constraints.B;
    auto u = input;

    return (initial.velocity + B * u / A) * mp::exp(A * time) - B * u / A;
  }

  /**
   * Calculates the time required to reach a specified velocity given the
   * initial velocity.
   *
   * @param velocity The goal velocity.
   * @param input The signed input applied to this profile from the initial
   *     state.
   * @param initial The initial velocity.
   * @return The time required to reach the goal velocity.
   */
  constexpr mp::quantity<mp::s> ComputeTimeFromVelocity(
      const Velocity_t& velocity, const Input_t& input,
      const Velocity_t& initial) const {
    auto A = m_constraints.A;
    auto B = m_constraints.B;
    auto u = input;

    return mp::log((A * velocity + B * u) / (A * initial + B * u)) / A;
  }

  /**
   * Calculates the distance reached at the same time as the given velocity when
   * applying the given input from the initial state.
   *
   * @param velocity The velocity reached by this profile
   * @param input The signed input applied to this profile from the initial
   *     state.
   * @param initial The initial state.
   * @return The distance reached when the given velocity is reached.
   */
  constexpr Distance_t ComputeDistanceFromVelocity(const Velocity_t& velocity,
                                                   const Input_t& input,
                                                   const State& initial) const {
    auto A = m_constraints.A;
    auto B = m_constraints.B;
    auto u = input;

    return initial.position + (velocity - initial.velocity) / A -
           B * u / (A * A) *
               mp::log((A * velocity + B * u) / (A * initial.velocity + B * u));
  }

  /**
   * Calculates the velocity at which input should be reversed in order to reach
   * the goal state from the current state.
   *
   * @param input The signed input applied to this profile from the current
   *     state.
   * @param current The current state.
   * @param goal The goal state.
   * @return The inflection velocity.
   */
  constexpr Velocity_t SolveForInflectionVelocity(const Input_t& input,
                                                  const State& current,
                                                  const State& goal) const {
    auto A = m_constraints.A;
    auto B = m_constraints.B;
    auto u = input;

    auto u_dir = u / mp::abs(u);

    auto position_delta = goal.position - current.position;
    auto velocity_delta = goal.velocity - current.velocity;

    auto scalar = (A * current.velocity + B * u) * (A * goal.velocity - B * u);
    auto power = -A / B / u * (A * position_delta - velocity_delta);

    auto a = -A * A;
    auto c = B * B * u * u + scalar * mp::exp(power);

    if (-1e-9 < mp::value(c) && mp::value(c) < 0) {
      // numeric instability - the heuristic gets it right but c is around
      // -1e-13
      return 0.0 * Velocity;
    }

    return u_dir * mp::sqrt(-c / a);
  }

  /**
   * Returns true if the profile should be inverted.
   *
   * The profile is inverted if we should first apply negative input in order to
   * reach the goal state.
   *
   * @param current The initial state (usually the current state).
   * @param goal The desired state when the profile is complete.
   */
  constexpr bool ShouldFlipInput(const State& current,
                                 const State& goal) const {
    auto u = m_constraints.maxInput;

    auto v0 = current.velocity;
    auto xf = goal.position;
    auto vf = goal.velocity;

    auto x_forward = ComputeDistanceFromVelocity(vf, u, current);
    auto x_reverse = ComputeDistanceFromVelocity(vf, -u, current);

    if (v0 >= m_constraints.MaxVelocity()) {
      return xf < x_reverse;
    }

    if (v0 <= -m_constraints.MaxVelocity()) {
      return xf < x_forward;
    }

    auto a = v0 >= 0.0 * Velocity;
    auto b = vf >= 0.0 * Velocity;
    auto c = xf >= x_forward;
    auto d = xf >= x_reverse;

    return (a && !d) || (b && !c) || (!c && !d);
  }

  Constraints m_constraints;
};

}  // namespace frc
