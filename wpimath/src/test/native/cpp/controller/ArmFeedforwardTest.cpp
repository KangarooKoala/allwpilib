// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "frc/EigenCore.h"
#include "frc/controller/ArmFeedforward.h"
#include "frc/system/NumericalIntegration.h"
#include "frc/units.h"

namespace {

using Ks_unit = mp::quantity<mp::V>;
using Kv_unit = mp::quantity<mp::V / (mp::rad / mp::s)>;
using Ka_unit = mp::quantity<mp::V / (mp::rad / mp::s2)>;
using Kg_unit = mp::quantity<mp::V>;

/**
 * Simulates a single-jointed arm and returns the final state.
 *
 * @param Ks The static gain, in volts.
 * @param Kv The velocity gain, in volt seconds per radian.
 * @param Ka The acceleration gain, in volt seconds² per radian.
 * @param Kg The gravity gain, in volts.
 * @param currentAngle The starting angle.
 * @param currentVelocity The starting angular velocity.
 * @param input The input voltage.
 * @param dt The simulation time.
 * @return The final state as a 2-vector of angle and angular velocity.
 */
frc::Matrixd<2, 1> Simulate(Ks_unit Ks, Kv_unit Kv, Ka_unit Ka, Kg_unit Kg,
                            mp::quantity<mp::rad> currentAngle,
                            mp::quantity<mp::rad / mp::s> currentVelocity,
                            mp::quantity<mp::V> input, mp::quantity<mp::s> dt) {
  frc::Matrixd<2, 2> A{{0.0, 1.0}, {0.0, -mp::value(Kv) / mp::value(Ka)}};
  frc::Matrixd<2, 1> B{{0.0}, {1.0 / mp::value(Ka)}};

  return frc::RK4(
      [&](const frc::Matrixd<2, 1>& x,
          const frc::Matrixd<1, 1>& u) -> frc::Matrixd<2, 1> {
        frc::Matrixd<2, 1> c{
            0.0, -mp::value(Ks) / mp::value(Ka) * wpi::sgn(x(1)) -
                     mp::value(Kg) / mp::value(Ka) * std::cos(x(0))};
        return A * x + B * u + c;
      },
      frc::Matrixd<2, 1>{mp::value(currentAngle), mp::value(currentVelocity)},
      frc::Matrixd<1, 1>{mp::value(input)}, dt);
}

/**
 * Simulates a single-jointed arm and returns the final state.
 *
 * @param armFF The feedforward object.
 * @param Ks The static gain, in volts.
 * @param Kv The velocity gain, in volt seconds per radian.
 * @param Ka The acceleration gain, in volt seconds² per radian.
 * @param Kg The gravity gain, in volts.
 * @param currentAngle The starting angle.
 * @param currentVelocity The starting angular velocity.
 * @param input The input voltage.
 * @param dt The simulation time.
 */
void CalculateAndSimulate(const frc::ArmFeedforward& armFF, Ks_unit Ks,
                          Kv_unit Kv, Ka_unit Ka, Kg_unit Kg,
                          mp::quantity<mp::rad> currentAngle,
                          mp::quantity<mp::rad / mp::s> currentVelocity,
                          mp::quantity<mp::rad / mp::s> nextVelocity,
                          mp::quantity<mp::s> dt) {
  auto input = armFF.Calculate(currentAngle, currentVelocity, nextVelocity);
  EXPECT_NEAR(
      mp::value(nextVelocity),
      Simulate(Ks, Kv, Ka, Kg, currentAngle, currentVelocity, input, dt)(1),
      1e-4);
}

}  // namespace

TEST(ArmFeedforwardTest, Calculate) {
  constexpr auto Ks = 0.5 * mp::V;
  constexpr auto Kv = 1.5 * mp::V / (mp::rad / mp::s);
  constexpr auto Ka = 2.0 * mp::V / (mp::rad / mp::s2);
  constexpr auto Kg = 1.0 * mp::V;
  frc::ArmFeedforward armFF{Ks, Kg, Kv, Ka};

  // Calculate(angle, angular velocity)
  EXPECT_NEAR(mp::value(armFF.Calculate(std::numbers::pi / 3.0 * mp::rad,
                                        0.0 * mp::rad / mp::s)),
              0.5, 0.002);
  EXPECT_NEAR(mp::value(armFF.Calculate(std::numbers::pi / 3.0 * mp::rad,
                                        1.0 * mp::rad / mp::s)),
              2.5, 0.002);

  // Calculate(currentAngle, currentVelocity, nextAngle, dt)
  CalculateAndSimulate(armFF, Ks, Kv, Ka, Kg, std::numbers::pi / 3.0 * mp::rad,
                       1.0 * mp::rad / mp::s, 1.05 * mp::rad / mp::s,
                       20.0 * mp::ms);
  CalculateAndSimulate(armFF, Ks, Kv, Ka, Kg, std::numbers::pi / 3.0 * mp::rad,
                       1.0 * mp::rad / mp::s, 0.95 * mp::rad / mp::s,
                       20.0 * mp::ms);
  CalculateAndSimulate(armFF, Ks, Kv, Ka, Kg, -std::numbers::pi / 3.0 * mp::rad,
                       1.0 * mp::rad / mp::s, 1.05 * mp::rad / mp::s,
                       20.0 * mp::ms);
  CalculateAndSimulate(armFF, Ks, Kv, Ka, Kg, -std::numbers::pi / 3.0 * mp::rad,
                       1.0 * mp::rad / mp::s, 0.95 * mp::rad / mp::s,
                       20.0 * mp::ms);
}

TEST(ArmFeedforwardTest, CalculateIllConditionedModel) {
  constexpr auto Ks = 0.39671 * mp::V;
  constexpr auto Kv = 2.7167 * mp::V / (mp::rad / mp::s);
  constexpr auto Ka = 1e-2 * mp::V / (mp::rad / mp::s2);
  constexpr auto Kg = 0.2708 * mp::V;
  frc::ArmFeedforward armFF{Ks, Kg, Kv, Ka};

  constexpr auto currentAngle = 1.0 * mp::rad;
  constexpr auto currentVelocity = 0.02 * mp::rad / mp::s;
  constexpr auto nextVelocity = 0.0 * mp::rad / mp::s;

  constexpr auto averageAccel =
      (nextVelocity - currentVelocity) / (20.0 * mp::ms);

  EXPECT_DOUBLE_EQ(
      mp::value(armFF.Calculate(currentAngle, currentVelocity, nextVelocity)),
      mp::value(Ks + Kv * currentVelocity + Ka * averageAccel +
                Kg * mp::cos(currentAngle)));
}

TEST(ArmFeedforwardTest, CalculateIllConditionedGradient) {
  constexpr auto Ks = 0.39671 * mp::V;
  constexpr auto Kv = 2.7167 * mp::V / (mp::rad / mp::s);
  constexpr auto Ka = 0.50799 * mp::V / (mp::rad / mp::s2);
  constexpr auto Kg = 0.2708 * mp::V;
  frc::ArmFeedforward armFF{Ks, Kg, Kv, Ka};

  CalculateAndSimulate(armFF, Ks, Kv, Ka, Kg, 1.0 * mp::rad,
                       0.02 * mp::rad / mp::s, 0.0 * mp::rad / mp::s,
                       20.0 * mp::ms);
}

TEST(ArmFeedforwardTest, AchievableVelocity) {
  constexpr auto Ks = 0.5 * mp::V;
  constexpr auto Kv = 1.5 * mp::V / (mp::rad / mp::s);
  constexpr auto Ka = 2.0 * mp::V / (mp::rad / mp::s2);
  constexpr auto Kg = 1.0 * mp::V;
  frc::ArmFeedforward armFF{Ks, Kg, Kv, Ka};

  EXPECT_NEAR(mp::value(armFF.MaxAchievableVelocity(
                  12.0 * mp::V, std::numbers::pi / 3.0 * mp::rad,
                  1.0 * mp::rad / mp::s2)),
              6, 0.002);
  EXPECT_NEAR(mp::value(armFF.MinAchievableVelocity(
                  11.5 * mp::V, std::numbers::pi / 3 * 1.0 * mp::rad,
                  1.0 * mp::rad / mp::s2)),
              -9, 0.002);
}

TEST(ArmFeedforwardTest, AchievableAcceleration) {
  constexpr auto Ks = 0.5 * mp::V;
  constexpr auto Kv = 1.5 * mp::V / (mp::rad / mp::s);
  constexpr auto Ka = 2.0 * mp::V / (mp::rad / mp::s2);
  constexpr auto Kg = 1.0 * mp::V;
  frc::ArmFeedforward armFF{Ks, Kg, Kv, Ka};

  EXPECT_NEAR(mp::value(armFF.MaxAchievableAcceleration(
                  12.0 * mp::V, std::numbers::pi / 3.0 * mp::rad,
                  1.0 * mp::rad / mp::s)),
              4.75, 0.002);
  EXPECT_NEAR(mp::value(armFF.MaxAchievableAcceleration(
                  12.0 * mp::V, std::numbers::pi / 3.0 * mp::rad,
                  -1.0 * mp::rad / mp::s)),
              6.75, 0.002);
  EXPECT_NEAR(mp::value(armFF.MinAchievableAcceleration(
                  12.0 * mp::V, std::numbers::pi / 3.0 * mp::rad,
                  1.0 * mp::rad / mp::s)),
              -7.25, 0.002);
  EXPECT_NEAR(mp::value(armFF.MinAchievableAcceleration(
                  12.0 * mp::V, std::numbers::pi / 3.0 * mp::rad,
                  -1.0 * mp::rad / mp::s)),
              -5.25, 0.002);
}

TEST(ArmFeedforwardTest, NegativeGains) {
  constexpr auto Ks = 0.5 * mp::V;
  constexpr auto Kv = 1.5 * mp::V / (mp::rad / mp::s);
  constexpr auto Ka = 2.0 * mp::V / (mp::rad / mp::s2);
  constexpr auto Kg = 1.0 * mp::V;
  frc::ArmFeedforward armFF{Ks, Kg, -Kv, -Ka};

  EXPECT_EQ(mp::value(armFF.GetKv()), 0);
  EXPECT_EQ(mp::value(armFF.GetKa()), 0);
}
