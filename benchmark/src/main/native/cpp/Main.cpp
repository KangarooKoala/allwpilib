// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <benchmark/benchmark.h>
#include <frc/geometry/Pose2d.h>
#include <frc/path/TravelingSalesman.h>
#include <frc/units.h>

#include <wpi/array.h>

static constexpr wpi::array<frc::Pose2d, 6> poses{
    frc::Pose2d{-1.0 * mp::m, 1.0 * mp::m, -90.0 * mp::deg},
    frc::Pose2d{-1.0 * mp::m, 2.0 * mp::m, 90.0 * mp::deg},
    frc::Pose2d{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::deg},
    frc::Pose2d{0.0 * mp::m, 3.0 * mp::m, -90.0 * mp::deg},
    frc::Pose2d{1.0 * mp::m, 1.0 * mp::m, 90.0 * mp::deg},
    frc::Pose2d{1.0 * mp::m, 2.0 * mp::m, 90.0 * mp::deg},
};
static constexpr int iterations = 100;

void BM_Transform(benchmark::State& state) {
  frc::TravelingSalesman traveler{[](auto pose1, auto pose2) {
    auto transform = pose2 - pose1;
    return mp::value(mp::hypot(transform.X(), transform.Y()));
  }};
  for (auto _ : state) {
    traveler.Solve(poses, iterations);
  }
}
BENCHMARK(BM_Transform);

void BM_Twist(benchmark::State& state) {
  frc::TravelingSalesman traveler{[](auto pose1, auto pose2) {
    auto twist = (pose2 - pose1).Log();
    return mp::value(mp::hypot(twist.dx, twist.dy));
  }};
  for (auto _ : state) {
    traveler.Solve(poses, iterations);
  }
}
BENCHMARK(BM_Twist);

BENCHMARK_MAIN();
