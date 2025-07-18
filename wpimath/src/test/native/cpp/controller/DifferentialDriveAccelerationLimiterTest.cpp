// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/controller/DifferentialDriveAccelerationLimiter.h"
#include "frc/system/plant/LinearSystemId.h"
#include "frc/units.h"

namespace frc {

TEST(DifferentialDriveAccelerationLimiterTest, LowLimits) {
  constexpr auto trackwidth = 0.9 * mp::m;
  constexpr auto dt = 5.0 * mp::ms;
  constexpr auto maxA = 2.0 * mp::m / mp::s2;
  constexpr auto maxAlpha = 2.0 * mp::rad / mp::s2;

  constexpr auto Kv_unit = mp::V / (mp::m / mp::s);
  constexpr auto Ka_unit = mp::V / (mp::m / mp::s2);
  auto plant = LinearSystemId::IdentifyDrivetrainSystem(
      1.0 * Kv_unit, 1.0 * Ka_unit, 1.0 * Kv_unit, 1.0 * Ka_unit);

  DifferentialDriveAccelerationLimiter accelLimiter{plant, trackwidth, maxA,
                                                    maxAlpha};

  Vectord<2> x{0.0, 0.0};
  Vectord<2> xAccelLimiter{0.0, 0.0};

  // Ensure voltage exceeds acceleration before limiting
  {
    Vectord<2> accels =
        plant.A() * xAccelLimiter + plant.B() * Vectord<2>{12.0, 12.0};
    mp::quantity<mp::m / mp::s2> a =
        (accels(0) + accels(1)) / 2.0 * mp::m / mp::s2;
    EXPECT_GT(mp::abs(a), maxA);
  }
  {
    Vectord<2> accels =
        plant.A() * xAccelLimiter + plant.B() * Vectord<2>{-12.0, 12.0};
    mp::quantity<mp::rad / mp::s2> alpha =
        (accels(1) - accels(0)) / mp::value(trackwidth) * mp::rad / mp::s2;
    EXPECT_GT(mp::abs(alpha), maxAlpha);
  }

  // Forward
  Vectord<2> u{12.0, 12.0};
  for (auto t = 0.0 * mp::s; t < 3.0 * mp::s; t += dt) {
    x = plant.CalculateX(x, u, dt);
    auto [left, right] = accelLimiter.Calculate(
        xAccelLimiter(0) * mp::m / mp::s, xAccelLimiter(1) * mp::m / mp::s,
        u(0) * mp::V, u(1) * mp::V);
    xAccelLimiter = plant.CalculateX(
        xAccelLimiter, Vectord<2>{mp::value(left), mp::value(right)}, dt);

    Vectord<2> accels =
        plant.A() * xAccelLimiter +
        plant.B() * Vectord<2>{mp::value(left), mp::value(right)};
    mp::quantity<mp::m / mp::s2> a =
        (accels(0) + accels(1)) / 2.0 * mp::m / mp::s2;
    mp::quantity<mp::rad / mp::s2> alpha =
        (accels(1) - accels(0)) / mp::value(trackwidth) * mp::rad / mp::s2;
    EXPECT_LE(mp::abs(a), maxA);
    EXPECT_LE(mp::abs(alpha), maxAlpha);
  }

  // Backward
  u = Vectord<2>{-12.0, -12.0};
  for (auto t = 0.0 * mp::s; t < 3.0 * mp::s; t += dt) {
    x = plant.CalculateX(x, u, dt);
    auto [left, right] = accelLimiter.Calculate(
        xAccelLimiter(0) * mp::m / mp::s, xAccelLimiter(1) * mp::m / mp::s,
        u(0) * mp::V, u(1) * mp::V);
    xAccelLimiter = plant.CalculateX(
        xAccelLimiter, Vectord<2>{mp::value(left), mp::value(right)}, dt);

    Vectord<2> accels =
        plant.A() * xAccelLimiter +
        plant.B() * Vectord<2>{mp::value(left), mp::value(right)};
    mp::quantity<mp::m / mp::s2> a =
        (accels(0) + accels(1)) / 2.0 * mp::m / mp::s2;
    mp::quantity<mp::rad / mp::s2> alpha =
        (accels(1) - accels(0)) / mp::value(trackwidth) * mp::rad / mp::s2;
    EXPECT_LE(mp::abs(a), maxA);
    EXPECT_LE(mp::abs(alpha), maxAlpha);
  }

  // Rotate CCW
  u = Vectord<2>{-12.0, 12.0};
  for (auto t = 0.0 * mp::s; t < 3.0 * mp::s; t += dt) {
    x = plant.CalculateX(x, u, dt);
    auto [left, right] = accelLimiter.Calculate(
        xAccelLimiter(0) * mp::m / mp::s, xAccelLimiter(1) * mp::m / mp::s,
        u(0) * mp::V, u(1) * mp::V);
    xAccelLimiter = plant.CalculateX(
        xAccelLimiter, Vectord<2>{mp::value(left), mp::value(right)}, dt);

    Vectord<2> accels =
        plant.A() * xAccelLimiter +
        plant.B() * Vectord<2>{mp::value(left), mp::value(right)};
    mp::quantity<mp::m / mp::s2> a =
        (accels(0) + accels(1)) / 2.0 * mp::m / mp::s2;
    mp::quantity<mp::rad / mp::s2> alpha =
        (accels(1) - accels(0)) / mp::value(trackwidth) * mp::rad / mp::s2;
    EXPECT_LE(mp::abs(a), maxA);
    EXPECT_LE(mp::abs(alpha), maxAlpha);
  }
}

TEST(DifferentialDriveAccelerationLimiterTest, HighLimits) {
  constexpr auto trackwidth = 0.9 * mp::m;
  constexpr auto dt = 5.0 * mp::ms;

  constexpr auto Kv_unit = mp::V / (mp::m / mp::s);
  constexpr auto Ka_unit = mp::V / (mp::m / mp::s2);

  auto plant = LinearSystemId::IdentifyDrivetrainSystem(
      1.0 * Kv_unit, 1.0 * Ka_unit, 1.0 * Kv_unit, 1.0 * Ka_unit);

  // Limits are so high, they don't get hit, so states of constrained and
  // unconstrained systems should match
  DifferentialDriveAccelerationLimiter accelLimiter{
      plant, trackwidth, 1'000.0 * mp::m / mp::s2, 1'000.0 * mp::rad / mp::s2};

  Vectord<2> x{0.0, 0.0};
  Vectord<2> xAccelLimiter{0.0, 0.0};

  // Forward
  Vectord<2> u{12.0, 12.0};
  for (auto t = 0.0 * mp::s; t < 3.0 * mp::s; t += dt) {
    x = plant.CalculateX(x, u, dt);
    auto [left, right] = accelLimiter.Calculate(
        xAccelLimiter(0) * mp::m / mp::s, xAccelLimiter(1) * mp::m / mp::s,
        u(0) * mp::V, u(1) * mp::V);
    xAccelLimiter = plant.CalculateX(
        xAccelLimiter, Vectord<2>{mp::value(left), mp::value(right)}, dt);

    EXPECT_DOUBLE_EQ(x(0), xAccelLimiter(0));
    EXPECT_DOUBLE_EQ(x(1), xAccelLimiter(1));
  }

  // Backward
  x.setZero();
  xAccelLimiter.setZero();
  u = Vectord<2>{-12.0, -12.0};
  for (auto t = 0.0 * mp::s; t < 3.0 * mp::s; t += dt) {
    x = plant.CalculateX(x, u, dt);
    auto [left, right] = accelLimiter.Calculate(
        xAccelLimiter(0) * mp::m / mp::s, xAccelLimiter(1) * mp::m / mp::s,
        u(0) * mp::V, u(1) * mp::V);
    xAccelLimiter = plant.CalculateX(
        xAccelLimiter, Vectord<2>{mp::value(left), mp::value(right)}, dt);

    EXPECT_DOUBLE_EQ(x(0), xAccelLimiter(0));
    EXPECT_DOUBLE_EQ(x(1), xAccelLimiter(1));
  }

  // Rotate CCW
  x.setZero();
  xAccelLimiter.setZero();
  u = Vectord<2>{-12.0, 12.0};
  for (auto t = 0.0 * mp::s; t < 3.0 * mp::s; t += dt) {
    x = plant.CalculateX(x, u, dt);
    auto [left, right] = accelLimiter.Calculate(
        xAccelLimiter(0) * mp::m / mp::s, xAccelLimiter(1) * mp::m / mp::s,
        u(0) * mp::V, u(1) * mp::V);
    xAccelLimiter = plant.CalculateX(
        xAccelLimiter, Vectord<2>{mp::value(left), mp::value(right)}, dt);

    EXPECT_DOUBLE_EQ(x(0), xAccelLimiter(0));
    EXPECT_DOUBLE_EQ(x(1), xAccelLimiter(1));
  }
}

TEST(DifferentialDriveAccelerationLimiterTest, SeparateMinMaxLowLimits) {
  constexpr auto trackwidth = 0.9 * mp::m;
  constexpr auto dt = 5.0 * mp::ms;
  constexpr auto minA = -1.0 * mp::m / mp::s2;
  constexpr auto maxA = 2.0 * mp::m / mp::s2;
  constexpr auto maxAlpha = 2.0 * mp::rad / mp::s2;

  constexpr auto Kv_unit = mp::V / (mp::m / mp::s);
  constexpr auto Ka_unit = mp::V / (mp::m / mp::s2);
  auto plant = LinearSystemId::IdentifyDrivetrainSystem(
      1.0 * Kv_unit, 1.0 * Ka_unit, 1.0 * Kv_unit, 1.0 * Ka_unit);

  DifferentialDriveAccelerationLimiter accelLimiter{plant, trackwidth, minA,
                                                    maxA, maxAlpha};

  Vectord<2> x{0.0, 0.0};
  Vectord<2> xAccelLimiter{0.0, 0.0};

  // Ensure voltage exceeds acceleration before limiting
  {
    Vectord<2> accels =
        plant.A() * xAccelLimiter + plant.B() * Vectord<2>{12.0, 12.0};
    mp::quantity<mp::m / mp::s2> a =
        (accels(0) + accels(1)) / 2.0 * mp::m / mp::s2;
    EXPECT_GT(mp::abs(a), maxA);
    EXPECT_GT(mp::abs(a), -minA);
  }

  // a should always be within [minA, maxA]
  // Forward
  Vectord<2> u{12.0, 12.0};
  for (auto t = 0.0 * mp::s; t < 3.0 * mp::s; t += dt) {
    x = plant.CalculateX(x, u, dt);
    auto [left, right] = accelLimiter.Calculate(
        xAccelLimiter(0) * mp::m / mp::s, xAccelLimiter(1) * mp::m / mp::s,
        u(0) * mp::V, u(1) * mp::V);
    xAccelLimiter = plant.CalculateX(
        xAccelLimiter, Vectord<2>{mp::value(left), mp::value(right)}, dt);

    Vectord<2> accels =
        plant.A() * xAccelLimiter +
        plant.B() * Vectord<2>{mp::value(left), mp::value(right)};
    mp::quantity<mp::m / mp::s2> a =
        (accels(0) + accels(1)) / 2.0 * mp::m / mp::s2;
    EXPECT_GE(a, minA);
    EXPECT_LE(a, maxA);
  }

  // Backward
  u = Vectord<2>{-12.0, -12.0};
  for (auto t = 0.0 * mp::s; t < 3.0 * mp::s; t += dt) {
    x = plant.CalculateX(x, u, dt);
    auto [left, right] = accelLimiter.Calculate(
        xAccelLimiter(0) * mp::m / mp::s, xAccelLimiter(1) * mp::m / mp::s,
        u(0) * mp::V, u(1) * mp::V);
    xAccelLimiter = plant.CalculateX(
        xAccelLimiter, Vectord<2>{mp::value(left), mp::value(right)}, dt);

    Vectord<2> accels =
        plant.A() * xAccelLimiter +
        plant.B() * Vectord<2>{mp::value(left), mp::value(right)};
    mp::quantity<mp::m / mp::s2> a =
        (accels(0) + accels(1)) / 2.0 * mp::m / mp::s2;
    EXPECT_GE(a, minA);
    EXPECT_LE(a, maxA);
  }
}

TEST(DifferentialDriveAccelerationLimiterTest, MinAccelGreaterThanMaxAccel) {
  constexpr auto Kv_unit = mp::V / (mp::m / mp::s);
  constexpr auto Ka_unit = mp::V / (mp::m / mp::s2);
  auto plant = LinearSystemId::IdentifyDrivetrainSystem(
      1.0 * Kv_unit, 1.0 * Ka_unit, 1.0 * Kv_unit, 1.0 * Ka_unit);
  EXPECT_NO_THROW({
    DifferentialDriveAccelerationLimiter accelLimiter(
        plant, 1.0 * mp::m, 1.0 * mp::m / mp::s2, 1.0 * mp::rad / mp::s2);
  });

  EXPECT_THROW(
      {
        DifferentialDriveAccelerationLimiter accelLimiter(
            plant, 1.0 * mp::m, 1.0 * mp::m / mp::s2, -1.0 * mp::m / mp::s2,
            1.0 * mp::rad / mp::s2);
      },
      std::invalid_argument);
}

}  // namespace frc
