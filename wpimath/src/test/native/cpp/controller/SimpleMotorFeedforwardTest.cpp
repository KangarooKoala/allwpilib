// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include <gtest/gtest.h>

#include "frc/EigenCore.h"
#include "frc/controller/LinearPlantInversionFeedforward.h"
#include "frc/controller/SimpleMotorFeedforward.h"
#include "frc/units.h"

namespace frc {

TEST(SimpleMotorFeedforwardTest, Calculate) {
  constexpr auto Ks = 0.5 * mp::V;
  constexpr auto Kv = 3.0 * mp::V / (mp::m / mp::s);
  constexpr auto Ka = 0.6 * mp::V / (mp::m / mp::s2);
  constexpr mp::quantity<mp::s> dt = 20.0 * mp::ms;

  constexpr Matrixd<1, 1> A{{-mp::value(Kv) / mp::value(Ka)}};
  constexpr Matrixd<1, 1> B{{1.0 / mp::value(Ka)}};

  frc::LinearPlantInversionFeedforward<1, 1> plantInversion{A, B, dt};
  frc::SimpleMotorFeedforward<mp::m> simpleMotor{Ks, Kv, Ka};

  constexpr Vectord<1> r{{2.0}};
  constexpr Vectord<1> nextR{{3.0}};

  EXPECT_NEAR(mp::value(37.524995834325161 * mp::V + Ks),
              mp::value(simpleMotor.Calculate(2.0 * mp::m / mp::s,
                                              3.0 * mp::m / mp::s)),
              0.002);
  EXPECT_NEAR(plantInversion.Calculate(r, nextR)(0) + mp::value(Ks),
              mp::value(simpleMotor.Calculate(2.0 * mp::m / mp::s,
                                              3.0 * mp::m / mp::s)),
              0.002);

  // These won't match exactly. It's just an approximation to make sure they're
  // in the same ballpark.
  EXPECT_NEAR(plantInversion.Calculate(r, nextR)(0) + mp::value(Ks),
              mp::value(simpleMotor.Calculate(2.0 * mp::m / mp::s,
                                              3.0 * mp::m / mp::s)),
              2.0);
}

TEST(SimpleMotorFeedforwardTest, NegativeGains) {
  constexpr auto Ks = 0.5 * mp::V;
  constexpr auto Kv = -3.0 * mp::V / (mp::m / mp::s);
  constexpr auto Ka = -0.6 * mp::V / (mp::m / mp::s2);
  constexpr mp::quantity<mp::s> dt = 0.0 * mp::ms;
  frc::SimpleMotorFeedforward<mp::m> simpleMotor{Ks, Kv, Ka, dt};
  EXPECT_EQ(mp::value(simpleMotor.GetKv()), 0);
  EXPECT_EQ(mp::value(simpleMotor.GetKa()), 0);
  EXPECT_EQ(mp::value(simpleMotor.GetDt()), 0.02);
}

}  // namespace frc
