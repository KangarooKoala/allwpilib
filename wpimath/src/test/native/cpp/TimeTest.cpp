// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <algorithm>
#include <chrono>
#include <cstddef>

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
  units::nanosecond_t total_duration = 0_ns;
  for (auto duration : durations) {
    total_duration += duration;
  }

  wpi::print("Calculating mean\n");
  units::nanosecond_t mean = total_duration / N;

  wpi::print("Calculating sum squares\n");
  auto sum_squares = 0_ns * 0_ns;
  for (auto duration : durations) {
    sum_squares += (duration - mean) * (duration - mean);
  }

  wpi::print("Calculating std dev\n");
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

  wpi::array<units::nanosecond_t, N> sorted{durations};
  std::sort(sorted.begin(), sorted.end());

  for (size_t i = 0; i < 10; ++i) {
    buffer[i] = sorted[i];
  }

  wpi::print("{}Fastest 10: {}\n", prefix, buffer);

  for (size_t i = 0; i < 10; ++i) {
    buffer[i] = sorted[N - 10 + i];
  }

  wpi::print("{}Slowest 10: {}\n", prefix, buffer);
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

  ProcessDurations(durations, prefix);
}

void TimeSuite(
    std::string_view name, std::function<void()> action,
    std::function<void()> setup = [] {}) {
  fmt::print("{}:\n", name);
  fmt::print("  Warmup: (100,000 iterations):\n");
  Time<100'000>(action, setup, "    ");
  for (size_t i = 0; i < 5; ++i) {
    fmt::print("  Run {}:\n", i);
    Time<1'000>(action, setup, "    ");
  }
}

TEST(TimeTest, Time) {
  wpi::print("Hello?\n");
}
