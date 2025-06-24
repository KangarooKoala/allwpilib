// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>
#include <sleipnir/autodiff/gradient.hpp>
#include <sleipnir/autodiff/hessian.hpp>
#include <sleipnir/optimization/problem.hpp>
#include <wpi/array.h>
#include <wpi/print.h>

#include "frc/estimator/DifferentialDrivePoseEstimator.h"
#include "frc/estimator/DifferentialDrivePoseEstimator3d.h"
#include "frc/geometry/Ellipse2d.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/kinematics/DifferentialDriveOdometry.h"
#include "frc/kinematics/DifferentialDriveOdometry3d.h"
#include "frc/kinematics/SD2.h"
#include "frc/kinematics/SwerveDesaturator.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "units/length.h"
#include "units/math.h"
#include "units/time.h"

template <typename T, size_t N, typename CharT>
struct fmt::formatter<wpi::array<T, N>, CharT> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return m_underlying.parse(ctx);
  }

  template <typename FmtContext>
  auto format(const wpi::array<T, N>& arr, FmtContext& ctx) const {
    auto out = ctx.out();

    out = fmt::format_to(out, "[");

    for (size_t i = 0; i < N; ++i) {
      out = m_underlying.format(arr[i], ctx);
      if (i < N - 1) {
        out = fmt::format_to(out, ", ");
      }
    }

    out = fmt::format_to(out, "]");

    return out;
  }

 private:
  fmt::formatter<T, CharT> m_underlying;
};

void ProcessDurations(const std::vector<units::nanosecond_t>& durations,
                      std::string_view prefix = "") {
  units::nanosecond_t total_duration = 0_ns;
  for (auto duration : durations) {
    total_duration += duration;
  }

  units::nanosecond_t mean = total_duration / durations.size();

  auto sum_squares = 0_ns * 0_ns;
  for (auto duration : durations) {
    sum_squares += (duration - mean) * (duration - mean);
  }

  units::nanosecond_t std_dev =
      units::math::sqrt(sum_squares / durations.size());

  wpi::print("{}Mean: {}, Std dev: {}\n", prefix, mean, std_dev);

  wpi::array<units::nanosecond_t, 10> buffer{wpi::empty_array};

  for (size_t i = 0; i < 10; ++i) {
    buffer[i] = durations[i];
  }

  wpi::print("{}First 10: {}\n", prefix, buffer);

  for (size_t i = 0; i < 10; ++i) {
    buffer[i] = durations[durations.size() - 10 + i];
  }

  wpi::print("{}Last 10: {}\n", prefix, buffer);

  std::vector<units::nanosecond_t> sorted{durations};
  std::sort(sorted.begin(), sorted.end());

  for (size_t i = 0; i < 10; ++i) {
    buffer[i] = sorted[i];
  }

  wpi::print("{}Fastest 10: {}\n", prefix, buffer);

  for (size_t i = 0; i < 10; ++i) {
    buffer[i] = sorted[durations.size() - 10 + i];
  }

  wpi::print("{}Slowest 10: {}\n", prefix, buffer);
}

template <size_t N>
void Time(
    std::function<void()> action, std::function<void()> setup = [] {},
    std::string_view prefix = "") {
  std::vector<units::nanosecond_t> durations{N};

  for (size_t i = 0; i < N; ++i) {
    setup();
    auto start = std::chrono::steady_clock::now();
    action();
    auto end = std::chrono::steady_clock::now();
    durations[i] = end - start;
  }

  ProcessDurations(durations, prefix);
}

void TimeSuite(
    std::string_view name, std::function<void()> action,
    std::function<void()> setup = [] {}) {
  fmt::print("{}:\n", name);
  // fmt::print("  Warmup: (100,000 iterations):\n");
  // Time<100'000>(action, setup, "    ");
  for (size_t i = 0; i < 5; ++i) {
    fmt::print("  Run {}:\n", i);
    Time<100>(action, setup, "    ");
  }
}

TEST(TimeTest, Time) {
  //  {
  //    frc::DifferentialDriveOdometry odometry{frc::Rotation2d{}, 0_m, 0_m,
  //                                            frc::Pose2d{}};
  //    frc::Rotation2d gyroAngle{};
  //    auto leftDistance = 0_m;
  //    auto rightDistance = 0_m;
  //    TimeSuite(
  //        "Odometry update (2d)",
  //        [&] { odometry.Update(gyroAngle, leftDistance, rightDistance); },
  //        [&] {
  //          odometry.ResetPosition(frc::Rotation2d{}, 0_m, 0_m,
  //          frc::Pose2d{});
  //        });
  //  }
  //
  //  {
  //    // Inputs
  //    double v_x = 4;
  //    double v_y = 0;
  //    double ω = 3;
  //    double Δt = 0.02;
  //    double v_max = 1;
  //
  //    double p_x = 0.3;
  //    double p_y = 0.3;
  //
  //    // Problem setup
  //    slp::OptimizationProblem problem;
  //
  //    slp::VariableMatrix v_input = {{v_x}, {v_y}};
  //    slp::VariableMatrix p_input_90_ccw = {{-p_y}, {p_x}};
  //    auto k = problem.DecisionVariable();
  //    auto ω_chassis = ω * k;
  //    auto halfΔθ = Δt / 2 * ω_chassis; // Really k ω Δt / 2, just written
  //    this way to optimize autodiff auto halfΔθTimesCotHalfΔθ = halfΔθ /
  //    slp::tan(halfΔθ); slp::VariableMatrix discretizationMatrix{
  //      {{ halfΔθTimesCotHalfΔθ, halfΔθ },
  //       { -halfΔθ, halfΔθTimesCotHalfΔθ }}};
  //    auto v_chassis = discretizationMatrix * k * v_input;
  //    auto v_module = v_chassis + ω_chassis * p_input_90_ccw;
  //
  //    problem.Maximize(k);
  //
  //    problem.SubjectTo(k > 0);
  //    problem.SubjectTo(k < 1);
  //    problem.SubjectTo(v_module.T() * v_module == v_max * v_max);
  //
  //    k.set_value(1);
  //    problem.Solve({.timeout = std::chrono::duration<double>{1}, .diagnostics
  //    = true});
  //
  //    wpi::print("Final solution: k = {}\n", k.value());
  //
  //    TimeSuite(
  //        "Sleipnir solve, subexpressions",
  //        [&] { k.set_value(1); problem.Solve(); },
  //        [&] {});
  //  }

  {
    frc::ChassisSpeeds continuousSpeeds{5_mps, -14_mps, 30_rad_per_s};
    units::second_t dt = 0.02_s;
    units::meters_per_second_t maxModuleSpeed = 1.3_mps;
    wpi::array<frc::Translation2d, 4> modules{
        frc::Translation2d{0.5_m, 0_m}, frc::Translation2d{0.5_m, 0_m},
        frc::Translation2d{0.5_m, 0.2_m}, frc::Translation2d{0.5_m, 0.2_m}};
    frc::SwerveDriveKinematics<4> kinematics{modules};

    TimeSuite(
        "Full solve",
        [&] {
          frc::SwerveDesaturator::DesaturatedDiscretize(
              continuousSpeeds, dt, maxModuleSpeed, modules);
        },
        [] {});

    TimeSuite(
        "SD3",
        [&] {
          sd2::DesaturatedDiscretize(continuousSpeeds, dt, maxModuleSpeed,
                                     modules);
        },
        [] {});

    TimeSuite(
        "Separate",
        [&] {
          frc::ChassisSpeeds discretized =
              frc::ChassisSpeeds::Discretize(continuousSpeeds, dt);
          wpi::array<frc::SwerveModuleState, 4> moduleStates =
              kinematics.ToSwerveModuleStates(discretized);
          kinematics.DesaturateWheelSpeeds(&moduleStates, maxModuleSpeed);
        },
        [] {});
  }

  return;

  {
    frc::ChassisSpeeds continuousSpeeds{4_mps, 0_mps, 3_rad_per_s};
    units::second_t dt = 0.02_s;
    units::meters_per_second_t maxModuleSpeed = 1_mps;
    wpi::array<frc::Translation2d, 4> modules{
        frc::Translation2d{1_m, 1_m}, frc::Translation2d{1_m, -1_m},
        frc::Translation2d{-1_m, 1_m}, frc::Translation2d{-1_m, -1_m}};

    auto f = [&] {
      auto discretizedSpeeds =
          frc::ChassisSpeeds::Discretize(continuousSpeeds, dt);
      for (auto module : modules) {
        double vx = discretizedSpeeds.vx.value() -
                    discretizedSpeeds.omega.value() * module.Y().value();
        double vy = discretizedSpeeds.vy.value() +
                    discretizedSpeeds.omega.value() * module.X().value();
        if (vx * vx + vy * vy >
            maxModuleSpeed.value() * maxModuleSpeed.value()) {
          return false;
        }
      }
      return true;
    };

    TimeSuite("Discretize check", [&] { f(); }, [&] {});
  }

  {
    double x = 2;

    auto f = [&] { return std::sqrt(std::sqrt(x)); };

    TimeSuite("sqrt(std::sqrt(x))", [&] { f(); }, [&] {});
  }

  {
    double x = 2;

    auto f = [&] { return std::pow(x, 0.25); };

    TimeSuite("pow(x, 0.25)", [&] { f(); }, [&] {});
  }

  {
    // Inputs
    double v_x = 4;
    double v_y = 0;
    double ω = 3;
    double Δt = 0.02;
    double v_max = 1;

    double p_x = 0.3;
    double p_y = 0.3;

    // Problem setup
    slp::VariableMatrix v_input = {{v_x}, {v_y}};
    slp::VariableMatrix p_input_90_ccw = {{-p_y}, {p_x}};
    slp::Variable k;
    auto halfΔθ =
        ω * Δt / 2 *
        k;  // Really k ω Δt / 2, just written this way to optimize autodiff
    auto halfΔθTimesCotHalfΔθ = halfΔθ / slp::tan(halfΔθ);
    slp::VariableMatrix discretizationMatrix{
        {{halfΔθTimesCotHalfΔθ, halfΔθ}, {-halfΔθ, halfΔθTimesCotHalfΔθ}}};
    auto v_chassis = discretizationMatrix * k * v_input;
    auto ω_chassis = ω * k;
    auto v_module = v_chassis + ω_chassis * p_input_90_ccw;
    auto error = (v_module.T() * v_module)(0, 0) - v_max * v_max;

    auto evaluate = [&] {
      k.set_value(1);

      return error.value();
    };

    TimeSuite("Sleipnir evaluate", [&] { evaluate(); }, [&] {});
  }

  {
    // Inputs
    double v_x = 4;
    double v_y = 0;
    double ω = 3;
    double Δt = 0.02;
    double v_max = 1;

    double p_x = 0.3;
    double p_y = 0.3;

    auto solve = [&] {
      // Problem setup
      slp::VariableMatrix v_input = {{v_x}, {v_y}};
      slp::VariableMatrix p_input_90_ccw = {{-p_y}, {p_x}};
      slp::Variable k;
      auto halfΔθ =
          ω * Δt / 2 *
          k;  // Really k ω Δt / 2, just written this way to optimize autodiff
      auto halfΔθTimesCotHalfΔθ = halfΔθ / slp::tan(halfΔθ);
      slp::VariableMatrix discretizationMatrix{
          {{halfΔθTimesCotHalfΔθ, halfΔθ}, {-halfΔθ, halfΔθTimesCotHalfΔθ}}};
      auto v_chassis = discretizationMatrix * k * v_input;
      auto ω_chassis = ω * k;
      auto v_module = v_chassis + ω_chassis * p_input_90_ccw;
      auto error = (v_module.T() * v_module)(0, 0) - v_max * v_max;
      auto derivative = slp::Gradient{error, k};

      // Newton's method
      double guess = 1;
      k.set_value(guess);
      double errorValue = error.value();
      while (errorValue > 1e-8) {
        guess -= errorValue / derivative.value().coeff(0);
        k.set_value(guess);
        errorValue = error.value();
      }
      return guess;
    };

    wpi::print("Final solution: k = {}\n", solve());

    TimeSuite("Newton's method v2", [&] { solve(); }, [&] {});
  }

  return;

  {
    // Inputs
    double v_x = 4;
    double v_y = 0;
    double ω = 3;
    double Δt = 0.02;
    double v_max = 1;

    double p_x = 0.3;
    double p_y = 0.3;

    auto solve = [&] {
      // Problem setup
      slp::VariableMatrix v_input = {{v_x}, {v_y}};
      slp::VariableMatrix p_input_90_ccw = {{-p_y}, {p_x}};
      slp::Variable k;
      auto halfΔθ =
          ω * Δt / 2 *
          k;  // Really k ω Δt / 2, just written this way to optimize autodiff
      auto halfΔθTimesCotHalfΔθ = halfΔθ / slp::tan(halfΔθ);
      slp::VariableMatrix discretizationMatrix{
          {{halfΔθTimesCotHalfΔθ, halfΔθ}, {-halfΔθ, halfΔθTimesCotHalfΔθ}}};
      auto v_chassis = discretizationMatrix * k * v_input;
      auto ω_chassis = ω * k;
      auto v_module = v_chassis + ω_chassis * p_input_90_ccw;
      auto error = (v_module.T() * v_module)(0, 0) - v_max * v_max;
      auto derivative = slp::Gradient{error, k};

      // Newton's method
      double guess = 1;
      k.set_value(guess);
      while (error.value() > 1e-8) {
        guess -= error.value() / derivative.value().coeff(0);
        k.set_value(guess);
      }
      return guess;
    };

    wpi::print("Final solution: k = {}\n", solve());

    TimeSuite("Newton's method v3", [&] { solve(); }, [&] {});
  }

  /*
  {
    frc::DifferentialDriveOdometry3d odometry{frc::Rotation3d{}, 0_m, 0_m,
                                              frc::Pose3d{}};
    frc::Rotation3d gyroAngle{};
    auto leftDistance = 0_m;
    auto rightDistance = 0_m;
    TimeSuite(
        "Odometry update (3d)",
        [&] { odometry.Update(gyroAngle, leftDistance, rightDistance); },
        [&] {
          odometry.ResetPosition(frc::Rotation3d{}, 0_m, 0_m, frc::Pose3d{});
        });
  }

  {
    frc::DifferentialDriveKinematics kinematics{1_m};
    frc::DifferentialDrivePoseEstimator poseEstimator{
        kinematics, frc::Rotation2d{}, 0_m, 0_m, frc::Pose2d{}};
    frc::Rotation2d gyroAngle{};
    auto leftDistance = 0_m;
    auto rightDistance = 0_m;
    TimeSuite(
        "Pose estimator update (2d)",
        [&] { poseEstimator.Update(gyroAngle, leftDistance, rightDistance); },
        [&] {
          poseEstimator.ResetPosition(frc::Rotation2d{}, 0_m, 0_m,
                                      frc::Pose2d{});
        });
  }

  {
    frc::DifferentialDriveKinematics kinematics{1_m};
    frc::DifferentialDrivePoseEstimator3d poseEstimator{
        kinematics, frc::Rotation3d{}, 0_m, 0_m, frc::Pose3d{}};
    frc::Rotation3d gyroAngle{};
    auto leftDistance = 0_m;
    auto rightDistance = 0_m;
    TimeSuite(
        "Pose estimator update (3d)",
        [&] { poseEstimator.Update(gyroAngle, leftDistance, rightDistance); },
        [&] {
          poseEstimator.ResetPosition(frc::Rotation3d{}, 0_m, 0_m,
                                      frc::Pose3d{});
        });
  }
  */
}
