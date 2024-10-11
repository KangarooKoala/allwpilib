// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>

#include <gtest/gtest.h>
#include <wpi/array.h>
#include <wpi/print.h>

#include "frc/estimator/DifferentialDrivePoseEstimator.h"
#include "frc/estimator/DifferentialDrivePoseEstimator3d.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/kinematics/DifferentialDriveOdometry.h"
#include "frc/kinematics/DifferentialDriveOdometry3d.h"
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
    wpi::print("Formatting wpi::array\n");
    auto out = ctx.out();

    out = fmt::format_to(out, "[");

    for (size_t i = 0; i < N; ++i) {
      out = m_underlying.format(arr[i], ctx);
      if (i < N - 1) {
        out = fmt::format_to(out, ", ");
      }
    }

    out = fmt::format_to(out, "]");

    wpi::print("Done formatting wpi::array\n");
    return out;
  }

 private:
  fmt::formatter<T, CharT> m_underlying;
};

template <size_t N>
void ProcessDurations(const wpi::array<units::nanosecond_t, N>& durations,
                      std::string_view prefix = "") {
  wpi::print("Summing durations\n");
  std::fflush(stdout);
  units::nanosecond_t total_duration = 0_ns;
  for (auto duration : durations) {
    total_duration += duration;
  }

  wpi::print("Calculating mean\n");
  std::fflush(stdout);
  units::nanosecond_t mean = total_duration / N;

  wpi::print("Calculating sum squares\n");
  std::fflush(stdout);
  auto sum_squares = 0_ns * 0_ns;
  for (auto duration : durations) {
    sum_squares += (duration - mean) * (duration - mean);
  }

  wpi::print("Calculating std dev\n");
  std::fflush(stdout);
  units::nanosecond_t std_dev = units::math::sqrt(sum_squares / N);

  wpi::print("{}Mean: {}, Std dev: {}\n", prefix, mean, std_dev);

  wpi::array<units::nanosecond_t, 10> buffer{wpi::empty_array};

  for (size_t i = 0; i < 10; ++i) {
    buffer[i] = durations[i];
  }

  wpi::print("{}First 10: {}\n", prefix, buffer);

  for (size_t i = 0; i < 10; ++i) {
    buffer[i] = durations[N - 10 + i];
  }

  wpi::print("{}Last 10: {}\n", prefix, buffer);
  std::fflush(stdout);

  wpi::array<units::nanosecond_t, 100'000> array_a(wpi::empty_array);
  for (size_t i = 0; i < 100'000; ++i) {
    array_a[i] = 0_ns;
  }

  [[ maybe_unused ]] std::array<units::nanosecond_t, 100'000> array_b{array_a};

  wpi::print("{}\n", array_b[0]);
}

template <size_t N>
void Time(
    std::function<void()> action, std::function<void()> setup = [] {},
    std::string_view prefix = "") {
  wpi::array<units::nanosecond_t, N> durations(wpi::empty_array);

  for (size_t i = 0; i < N; ++i) {
    setup();
    auto start = std::chrono::steady_clock::now();
    action();
    auto end = std::chrono::steady_clock::now();
    durations[i] = end - start;
  }

  fmt::print("Processing durations\n");
  std::fflush(stdout);
  ProcessDurations<N>(durations, prefix);
}

void TimeSuite(
    std::string_view name, std::function<void()> action,
    std::function<void()> setup = [] {}) {
  fmt::print("{}:\n", name);
  fmt::print("  Warmup: (100,000 iterations):\n");
  std::fflush(stdout);
  Time<100'000>(action, setup, "    ");
  for (size_t i = 0; i < 5; ++i) {
    fmt::print("  Run {}:\n", i);
    std::fflush(stdout);
    Time<1'000>(action, setup, "    ");
  }
}

TEST(TimeTest, Time) {
  {
    frc::DifferentialDriveOdometry odometry{frc::Rotation2d{}, 0_m, 0_m,
                                            frc::Pose2d{}};
    frc::Rotation2d gyroAngle{};
    auto leftDistance = 0_m;
    auto rightDistance = 0_m;
    wpi::print("Running time suite\n");
    std::fflush(stdout);
    TimeSuite(
        "Odometry update (2d)",
        [&] { odometry.Update(gyroAngle, leftDistance, rightDistance); },
        [&] {
          odometry.ResetPosition(frc::Rotation2d{}, 0_m, 0_m, frc::Pose2d{});
        });
  }

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
}
