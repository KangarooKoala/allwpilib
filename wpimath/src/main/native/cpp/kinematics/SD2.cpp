// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/kinematics/SD2.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <gcem.hpp>
#include <wpi/print.h>

using namespace frc;

constexpr bool IsZero(double x) {
  return gcem::abs(x) < 1e-9;
}

ChassisSpeeds sd2::DesaturatedDiscretize(
    const ChassisSpeeds& continuousSpeeds, units::second_t dt,
    units::meters_per_second_t maxModuleSpeed,
    std::span<const Translation2d> modules, bool debug, bool v3) {
  double vx = continuousSpeeds.vx.value();
  double vy = continuousSpeeds.vy.value();
  double ω = continuousSpeeds.omega.value();
  double Δt = dt.value();
  double v_max = maxModuleSpeed.value();
  [[maybe_unused]]
  bool fDebug = false;

  int functionCalls = 0;

  auto error = [&](double px, double py, double k) {
    ++functionCalls;
    double halfΔθ = ω * Δt / 2 * k;
    double halfΔθByTanHalfΔθ;
    if (!IsZero(halfΔθ)) {
      halfΔθByTanHalfΔθ = halfΔθ / std::tan(halfΔθ);
    } else {
      double halfΔθSq = halfΔθ * halfΔθ;
      halfΔθByTanHalfΔθ =
          1.0 - 1.0 / 3.0 * halfΔθSq - 1.0 / 45.0 * halfΔθSq * halfΔθSq;
    }
    double v_chassis_x = halfΔθByTanHalfΔθ * k * vx + halfΔθ * k * vy;
    double v_chassis_y = -halfΔθ * k * vx + halfΔθByTanHalfΔθ * k * vy;
    double ω_chassis = k * ω;
    double v_module_x = v_chassis_x - ω_chassis * py;
    double v_module_y = v_chassis_y + ω_chassis * px;
    return (v_module_x * v_module_x + v_module_y * v_module_y) - v_max * v_max;
  };

  double maxK = 1;

  // It's possible for a valid k for one module to be invalid for another
  // module, even if that module has a greater valid k. (Sometimes, scaling down
  // the speeds *increases* a module's velocity because the discretized chassis
  // translational velocity better aligns with the module's velocity from the
  // chassis rotational velocity.) Therefore, check previous modules again if
  // we've changed k.

  // The initial value just needs to be different from a valid index
  size_t firstVerifiedIndex = modules.size();
  for (size_t i = 0; i != firstVerifiedIndex; i = (i + 1) % modules.size()) {
    Translation2d module = modules[i];
    double px = module.X().value();
    double py = module.Y().value();

    double k = maxK;
    double y = error(px, py, k);
    if (debug) {
      wpi::print("Initial k = {}, y = {}\n", k, y);
    }
    if (IsZero(y) || y < 0) {
      if (firstVerifiedIndex >= modules.size()) {
        firstVerifiedIndex = i;
      }
      if (debug) {
        wpi::print("Valid y, skipping\n");
      }
      continue;
    }

    double k_prev = k;
    double y_prev = y;
    k = k_prev - 1e-9;
    y = error(px, py, k);

    if (!v3) {
      // Apply secant method
      int secantIters = 0;
      while (!IsZero(y)) {
        double secant = (y - y_prev) / (k - k_prev);
        double k_new;
        if (IsZero(secant) || secant < 0) {
          // Only one root with positive slope between k and 0, so we can halve
          // k
          if (debug) {
            wpi::print("Non-positive slope! Skipping to {}\n", 0.5 * k);
          }
          k_new = 0.5 * k;
        } else {
          // TODO Consider using regula falsi
          k_new = k - y / secant;
          // TODO Rigorously justify
          double min_k = 0.5 * k;
          double max_k = 2.0 * k;
          if (k_new < min_k || k_new > max_k) {
            double clamped_k = std::clamp(k_new, min_k, max_k);
            if (debug) {
              wpi::print("Overshoot to {}! Limiting to {}\n", k_new, clamped_k);
            }
            k_new = clamped_k;
          }
        }
        k_prev = k;
        y_prev = y;
        k = k_new;
        y = error(px, py, k);
        ++secantIters;
      }
      if (debug) {
        wpi::print("Found solution {} in {} iterations\n", k, secantIters);
      }
    } else {
      // Secant method
      int secantIters = 0;
      while (!IsZero(y) && y > 0) {
        ++secantIters;
        double secant = (y - y_prev) / (k - k_prev);
        double k_new;
        if (IsZero(secant) || secant < 0) {
          // Only one root with positive slope between k and 0, so we can halve
          // k
          if (debug) {
            wpi::print("Non-positive slope! Skipping to {}\n", 0.5 * k);
          }
          k_new = 0.5 * k;
        } else {
          k_new = k - y / secant;
          // TODO Compress
          if (k_new < 0.5 * k) {
            if (debug) {
              wpi::print("Overshoot to {}! Limiting to {}\n", k_new, 0.5 * k);
            }
            k_new = 0.5 * k;
          }
        }
        k_prev = k;
        y_prev = y;
        k = k_new;
        y = error(px, py, k);
      }
      if (debug) {
        wpi::print("Reached k = {} in {} secant iterations\n", k, secantIters);
      }

      if (!IsZero(y)) {
        double k_neg = k;
        double y_neg = y;
        double k_pos = k_prev;
        double y_pos = y_prev;

        int regulaFalsiIters = 0;
        while (true) {
          ++regulaFalsiIters;
          double k_new = (k_neg * y_pos - k_pos * y_neg) / (y_pos - y_neg);
          double y_new = error(px, py, k_new);
          if (IsZero(y_new)) {
            k = k_new;
            y = y_new;
            break;
          }
          if (y_new < 0) {
            k_neg = k_new;
            y_neg = y_new;
          } else {
            k_pos = k_new;
            y_pos = y_new;
          }
        }
        if (debug) {
          wpi::print("Reached k = {} in {} regula falsi iterations\n", k,
                     regulaFalsiIters);
        }
      }
    }

    // Problem statement
    //  * We are trying to find the greatest k ∈ [0, max_k] such that f(k) ≤ 0,
    //    which is max_k if f(max_k) ≤ 0 and otherwise the greatest root of
    //    f(k) in [0, max_k] with a positive slope
    //
    // Properties
    // Assume the following properties of f(k):
    //   (1) f(k) has at most 4 roots
    //   (2) If f(k) has 4 roots, f'(k) ≥ 0 and f''(k) ≥ 0 to the right of the
    //       greatest root
    // (1) and (2) are justified as reasonable because f(k) can be very well
    // approximated as a polynomial with maximum degree of 4 (Note that
    // f⁽⁵⁾(0) = 0)
    //
    // (1) implies that there is at most 2 roots with positive slope
    //
    // Demonstrating convergence
    //
    // From (2), we know that if f(k) has 2 roots with positive slope and max_k
    // is to the right of the greatest root, the secant method will converge to
    // the correct root (because every iteration will produce a new k between
    // the correct root and the current guess)
    // Furthermore, our modified secant method will also converge
    //
    // In any other case, there is exact one root with positive slope in the
    // range [0, max_k]
    //
    // We preserve the property that during the secant method, f(k) > 0, which
    // lets us constrain k_new to [k/2, k], which keeps us in [0, max_k], so
    // we cannot converge to an incorrect root (since the only root in that
    // range is the correct root)
    // If we do not converge to a root during the secant method, we then use
    // the regula falsi (or false position method), which constrains guesses
    // to be between the previous two, and since we know there is only one
    // root, we know it will converge to the correct root

    maxK = k;
    firstVerifiedIndex = i;
  }

  if (debug) {
    wpi::print("maxK = {}, function calls = {}\n", maxK, functionCalls);
  }

  return ChassisSpeeds::Discretize(maxK * vx * 1_mps, maxK * vy * 1_mps,
                                   maxK * ω * 1_rad_per_s, dt);
}
