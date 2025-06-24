// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/kinematics/SwerveDesaturator.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <gcem.hpp>
#include <wpi/print.h>

#include "frc/kinematics/SwerveModuleState.h"
#include "units/angular_velocity.h"

using namespace frc;

template <typename T, typename CharT>
struct fmt::formatter<std::span<T>, CharT> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return m_underlying.parse(ctx);
  }

  template <typename FmtContext>
  auto format(const std::span<T>& span, FmtContext& ctx) const {
    auto out = ctx.out();

    out = fmt::format_to(out, "[");
    bool isFirst = true;
    for (auto x : span) {
      if (isFirst) {
        isFirst = false;
      } else {
        out = fmt::format_to(out, ", ");
      }
      out = m_underlying.format(x, ctx);
    }
    out = fmt::format_to(out, "]");

    return out;
  }

 private:
  fmt::formatter<T, CharT> m_underlying;
};

template <typename T, typename CharT>
struct fmt::formatter<std::pair<T, T>, CharT> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return m_underlying.parse(ctx);
  }

  template <typename FmtContext>
  auto format(const std::pair<T, T>& pair, FmtContext& ctx) const {
    auto out = ctx.out();

    out = fmt::format_to(out, "(");
    out = m_underlying.format(pair.first, ctx);
    out = fmt::format_to(out, ", ");
    out = m_underlying.format(pair.second, ctx);
    out = fmt::format_to(out, ")");

    return out;
  }

 private:
  fmt::formatter<T, CharT> m_underlying;
};

constexpr bool IsZero(double x) {
  return gcem::abs(x) < 1e-9;
}

std::vector<double> GetInitialGuesses(double a4, double a3, double a2,
                                      std::function<double(double)>&& f) {
  // p(x) = a₄ x⁴ + a₃ x³ + a₂ x² + a₀
  // p'(x) = 4 a₄ x³ + 3 a₃ x² + 2 a₂ x
  //       = x (4 a₄ x² + 3 a₃ x + 2 a₂)
  // p''(x) = 12 a₄ x² + 6 a₃ x + 2 a₂
  //        = 2 (6 a₄ x² + 3 a₃ x + a₂)

  std::vector<double> criticalPoints;
  std::vector<double> inflectionPoints;
  criticalPoints.reserve(3);
  inflectionPoints.reserve(2);

  criticalPoints.emplace_back(0);

  if (IsZero(a4)) {
    if (IsZero(a3)) {
      if (IsZero(a2)) {
        // f(x) = a₀ ≤ 0, so x ∈ [0, 1] is valid. Everything in validRanges will
        // continue to be valid, so skip to the next module.
        return std::vector<double>{};
      }
      // f(x) = a₂ x² + a₀, a₂ ≠ 0
      // Exactly one critical point at 0, which is already included.
      // No inflection points to add
    } else {
      // f'(x) = 3 a₃ x² + 2 a₂ x
      // f'(x) = 0, x ≠ 0:
      //   3 a₃ x² + 2 a₂ x = 0
      //   3 a₃ x + 2 a₂ = 0
      //   3 a₃ x = -2 a₂
      //   x = -2 a₂ / 3 a₃
      criticalPoints.emplace_back(-2 * a2 / (3 * a3));
      // f''(x) = 6 a₃ x + 2 a₂
      // f''(x) = 0:
      //   6 a₃ x + 2 a₂ = 0
      //   6 a₃ x = -2 a₂
      //   x = -a₂ / 3 a₃
      inflectionPoints.emplace_back(-a2 / (3 * a3));
    }
  } else {
    {
      // f'(x) = 4 a₄ x³ + 3 a₃ x² + 2 a₂ x
      // f'(x) = 0, x ≠ 0:
      //   4 a₄ x³ + 3 a₃ x² + 2 a₂ x = 0
      //   4 a₄ x² + 3 a₃ x + 2 a₂ = 0
      //       -3 a₃    ⎛⎛-3 a₃⎞²   2 a₂⎞
      //   x = ――――― ± √⎜⎜―――――⎟  - ――――⎟
      //       8 a₄     ⎝⎝8 a₄ ⎠    4 a₄⎠
      //   m := -3 a₃ / 8 a₄
      //   q := 2 a₂ / 4 a₄ = a₂ / 2 a₄
      //   x = m ± √(m * m - q)
      double m = -3 * a3 / (8 * a4);
      double q = a2 / (2 * a4);
      double D = m * m - q;
      // Check IsZero() first to make sure we recognize small positive numbers
      // as 0
      if (IsZero(D)) {
        criticalPoints.emplace_back(m);
      } else if (D > 0) {
        criticalPoints.emplace_back(m - std::sqrt(D));
        criticalPoints.emplace_back(m + std::sqrt(D));
      }
    }
    {
      // f''(x) = 12 a₄ x² + 6 a₃ x + 2 a₂
      // f'(x) = 0, x ≠ 0:
      //   12 a₄ x² + 6 a₃ x + 2 a₂ = 0
      //       -6 a₃    ⎛⎛-6 a₃⎞²    2 a₂⎞
      //   x = ――――― ± √⎜⎜―――――⎟  - ―――――⎟
      //       24 a₄    ⎝⎝12 a₄⎠    12 a₄⎠
      //   m := -6 a₃ / 24 a₄ = -a₃ / 4 a₄
      //   q := 2 a₂ / 12 a₄ = a₂ / 6 a₄
      //   x = m ± √(m * m - q)
      double m = -a3 / (4 * a4);
      double q = a2 / (6 * a4);
      double D = m * m - q;
      // Check IsZero() first to make sure we recognize small positive numbers
      // as 0
      if (IsZero(D)) {
        inflectionPoints.emplace_back(m);
      } else if (D > 0) {
        inflectionPoints.emplace_back(m - std::sqrt(D));
        inflectionPoints.emplace_back(m + std::sqrt(D));
      }
    }
  }

  std::sort(criticalPoints.begin(), criticalPoints.end());
  std::sort(inflectionPoints.begin(), inflectionPoints.end());

  std::vector<double> criticalValues;
  criticalValues.reserve(criticalPoints.size());

  for (double cp : criticalPoints) {
    criticalValues.emplace_back(f(cp));
  }
  std::vector<double> guesses;
  guesses.reserve(criticalPoints.size());

  // TODO Should filtering be done on the critical points before adding
  // criticalValues?
  for (size_t i = 0; i < criticalPoints.size() - 1; ++i) {
    if (criticalPoints[i] > 1 && !IsZero(criticalPoints[i] - 1)) {
      // First critical point is greater than 1, so this root (and any after)
      // must be greater than 1
      return guesses;
    }
    if (criticalPoints[i + 1] < 0 || IsZero(criticalPoints[i + 1])) {
      // Second critical point is strictly less than 0, so this root must be
      // less than 0
      continue;
    }
    double left = criticalValues[i];
    double right = criticalValues[i + 1];
    if (IsZero(left)) {
      guesses.emplace_back(criticalPoints[i]);
      continue;
    }
    if (IsZero(right)) {
      // Process in the next loop
      continue;
    }
    // Signs are different, guaranteed a root in here
    if (left * right < 0) {
      guesses.emplace_back(inflectionPoints[i]);
    }
  }

  // Handle last root
  // The calls to .back() are safe because we are guaranteed a critical value at
  // x = 0, so the vectors will never be empty

  if (criticalPoints.back() > 1 && !IsZero(criticalPoints.back() - 1)) {
    return guesses;
  }

  if (IsZero(criticalValues.back())) {
    guesses.emplace_back(criticalPoints.back());
  } else if (!IsZero(a4) && criticalValues.back() * a4 < 0) {
    // f(x) = 24 a₄ (x - x_cp)⁴ + f(x_cp)
    // f(x) = 0:
    //   6 a₄ (x - x_cp)⁴ + f(x_cp) = 0
    //   (x - x_cp)⁴ = -f(x_cp) / 24 a₄
    //   x - x_cp = ∜(-f(x_cp) / 24 a₄)
    //   x = x_cp + ∜(-f(x_cp) / 24 a₄)
    [[maybe_unused]]
    double initialGuess = criticalPoints.back() +
                          std::pow(-criticalValues.back() / (24 * a4), 0.25);
    guesses.emplace_back(initialGuess);
  } else if (IsZero(a4) && !IsZero(a3) && criticalValues.back() * a3 < 0) {
    // This shouldn't be possible at all mathematically, but include this just
    // in case f(x) = 6 a₃ (x - x_cp)³ + f(x_cp) f(x) = 0:
    //   6 a₃ (x - x_cp)³ + f(x_cp) = 0
    //   (x - x_cp)³ = -f(x_cp) / 6 a₃
    //   x - x_cp = ∛(-f(x_cp) / 6 a₃)
    //   x = x_cp + ∛(-f(x_cp) / 6 a₃)
    double initialGuess =
        criticalPoints.back() + std::cbrt(-criticalValues.back() / (6 * a3));
    guesses.emplace_back(initialGuess);
  } else if (IsZero(a4) && IsZero(a3) && !IsZero(a2) &&
             criticalValues.back() * a2 < 0) {
    // f(x) = 2 a₂ (x - x_cp)² + f(x_cp)
    // f(x) = 0:
    //   2 a₂ (x - x_cp)² + f(x_cp) = 0
    //   (x - x_cp)² = -f(x_cp) / 2 a₂
    //   x - x_cp = √(-f(x_cp) / 2 a₂)
    //   x = x_cp + √(-f(x_cp) / 2 a₂)
    double initialGuess =
        criticalPoints.back() + std::sqrt(-criticalValues.back() / (2 * a2));
    guesses.emplace_back(initialGuess);
  }

  return guesses;
}

void UpdateValidRanges(
    std::vector<std::pair<double, double>>& validRanges,
    const std::vector<std::pair<double, double>>& moduleRanges, bool debug) {
  auto it = validRanges.begin();
  for (auto [start, end] : moduleRanges) {
    // Discard ranges not in a previous range that are completely before this
    // one
    while (it < validRanges.end() && it->second < start) {
      it = validRanges.erase(it);
    }
    // Clamp the first range to be within the current module range
    if (it < validRanges.end() && it->first < start) {
      it->first = start;
    }
    // Keep ranges completely within this one
    while (it < validRanges.end() && it->second < end) {
      ++it;
    }
    // Split the last range at the end of the current module range
    if (it < validRanges.end() && it->first < end && it->second > end) {
      double validRangeEnd = it->second;
      it->second = end;
      ++it;
      it = validRanges.emplace(it, end, validRangeEnd);
      // Decrement so that we don't move on past the new range
      --it;
    }
    // Move on to the next ranges
    if (it < validRanges.end()) {
      ++it;
    }
  }
  // Remove any ranges after the module ranges
  validRanges.erase(it, validRanges.end());
}

ChassisSpeeds SwerveDesaturator::DesaturatedDiscretize(
    const ChassisSpeeds& continuousSpeeds, units::second_t dt,
    units::meters_per_second_t maxModuleSpeed,
    std::span<const Translation2d> modules, bool debug) {
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

  auto secant = [&](double px, double py, double k) {
    return (error(px, py, k + 1e-9) - error(px, py, k)) * 1e9;
  };

  std::vector<std::pair<double, double>> validRanges;
  validRanges.emplace_back(0, 1);

  for (Translation2d module : modules) {
    double px = module.X().value();
    double py = module.Y().value();

    double v_mag_sq = vx * vx + vy * vy;
    double v_dot_p = vx * px + vy * py;
    double v_dot_p_90_ccw = vx * -py + vy * px;
    double p_mag_sq = px * px + py * py;
    double halfOriginalΔθ = ω * Δt / 2;
    double halfOriginalΔθSq = halfOriginalΔθ * halfOriginalΔθ;

    double a4 = v_mag_sq * (1.0 / 3) * halfOriginalΔθSq -
                2 * ω * v_dot_p_90_ccw * (1.0 / 3) * halfOriginalΔθSq;
    double a3 = 2 * ω * -v_dot_p * halfOriginalΔθ;
    double a2 = v_mag_sq + 2 * ω * v_dot_p_90_ccw + ω * ω * p_mag_sq;
    [[maybe_unused]]
    double a0 = -v_max * v_max;

    if (debug) {
      wpi::print("a4 = {}, a3 = {}, a2 = {}, a0 = {}\n", a4, a3, a2, a0);
    }

    std::vector<double> guesses = GetInitialGuesses(
        a4, a3, a2, [&](double k) { return error(px, py, k); });

    std::vector<double> roots;
    roots.reserve(guesses.size());

    for (double guess : guesses) {
      int iterCount = 0;
      if (debug) {
        wpi::print("Initial fine guess: {}\n", guess);
      }
      double errorValue = error(px, py, guess);
      while (!IsZero(errorValue)) {
        double secantValue = secant(px, py, guess);
        guess -= errorValue / secantValue;
        errorValue = error(px, py, guess);
        ++iterCount;
      }
      if (debug) {
        wpi::print("Found solution {} in {} iterations\n", guess, iterCount);
      }
      roots.emplace_back(guess);
    }

    // TODO Verify when a0 == 0
    // TODO Does this need explanation?
    std::vector<std::pair<double, double>> moduleRanges;
    if (roots.size() > 0) {
      moduleRanges.emplace_back(0, roots[0]);
      if (roots.size() > 2) {
        moduleRanges.emplace_back(roots[1], roots[2]);
      }
    }

    if (debug) {
      wpi::print("Guesses: {}; Roots: {}\n", std::span{guesses},
                 std::span{roots});
      // wpi::print(
      //   "Critical points: {}; Inflection points: {}; Guesses: {}; Roots:
      //   {}\n", std::span{criticalPoints}, std::span{inflectionPoints},
      //   std::span{guesses},
      //   std::span{roots});
      wpi::print("Module ranges: [");
      for (auto [start, end] : moduleRanges) {
        wpi::print("[{}, {}], ", start, end);
      }
      wpi::print("]\n");
    }

    UpdateValidRanges(validRanges, moduleRanges, debug);

    if (debug) {
      wpi::print("Valid ranges: {}\n", std::span{validRanges});
    }
  }

  double maxK = validRanges.back().second;

  if (debug) {
    wpi::print("maxK = {}, function calls = {}\n", maxK, functionCalls);
  }

  return ChassisSpeeds::Discretize(maxK * vx * 1_mps, maxK * vy * 1_mps,
                                   maxK * ω * 1_rad_per_s, dt);
}
