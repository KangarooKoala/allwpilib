// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include <gtest/gtest.h>

#include "frc/EigenCore.h"
#include "frc/controller/ElevatorFeedforward.h"
#include "frc/controller/LinearPlantInversionFeedforward.h"
#include "frc/units.h"

static constexpr auto Ks = 0.5 * mp::V;
static constexpr auto Kv = 1.5 * mp::V / (mp::m / mp::s);
static constexpr auto Ka = 2.0 * mp::V / (mp::m / mp::s2);
static constexpr auto Kg = 1.0 * mp::V;

TEST(ElevatorFeedforwardTest, Calculate) {
  frc::ElevatorFeedforward elevatorFF{Ks, Kg, Kv, Ka};

  EXPECT_NEAR(mp::value(elevatorFF.Calculate(0.0 * mp::m / mp::s)),
              mp::value(Kg), 0.002);
  EXPECT_NEAR(mp::value(elevatorFF.Calculate(2.0 * mp::m / mp::s)), 4.5, 0.002);

  frc::Matrixd<1, 1> A{-mp::value(Kv) / mp::value(Ka)};
  frc::Matrixd<1, 1> B{1.0 / mp::value(Ka)};
  constexpr mp::quantity<mp::s> dt = 20.0 * mp::ms;
  frc::LinearPlantInversionFeedforward<1, 1> plantInversion{A, B, dt};

  frc::Vectord<1> r{2.0};
  frc::Vectord<1> nextR{3.0};
  EXPECT_NEAR(
      plantInversion.Calculate(r, nextR)(0) + mp::value(Ks) + mp::value(Kg),
      mp::value(elevatorFF.Calculate(2.0 * mp::m / mp::s, 3.0 * mp::m / mp::s)),
      0.002);
}

TEST(ElevatorFeedforwardTest, AchievableVelocity) {
  frc::ElevatorFeedforward elevatorFF{Ks, Kg, Kv, Ka};
  EXPECT_NEAR(mp::value(elevatorFF.MaxAchievableVelocity(11.0 * mp::V,
                                                         1.0 * mp::m / mp::s2)),
              5, 0.002);
  EXPECT_NEAR(mp::value(elevatorFF.MinAchievableVelocity(11.0 * mp::V,
                                                         1.0 * mp::m / mp::s2)),
              -9, 0.002);
}

TEST(ElevatorFeedforwardTest, AchievableAcceleration) {
  frc::ElevatorFeedforward elevatorFF{Ks, Kg, Kv, Ka};
  EXPECT_NEAR(mp::value(elevatorFF.MaxAchievableAcceleration(
                  12.0 * mp::V, 2.0 * mp::m / mp::s)),
              3.75, 0.002);
  EXPECT_NEAR(mp::value(elevatorFF.MaxAchievableAcceleration(
                  12.0 * mp::V, -2.0 * mp::m / mp::s)),
              7.25, 0.002);
  EXPECT_NEAR(mp::value(elevatorFF.MinAchievableAcceleration(
                  12.0 * mp::V, 2.0 * mp::m / mp::s)),
              -8.25, 0.002);
  EXPECT_NEAR(mp::value(elevatorFF.MinAchievableAcceleration(
                  12.0 * mp::V, -2.0 * mp::m / mp::s)),
              -4.75, 0.002);
}

TEST(ElevatorFeedforwardTest, NegativeGains) {
  frc::ElevatorFeedforward elevatorFF{Ks, Kg, -Kv, -Ka};
  EXPECT_EQ(mp::value(elevatorFF.GetKv()), 0);
  EXPECT_EQ(mp::value(elevatorFF.GetKa()), 0);
}
