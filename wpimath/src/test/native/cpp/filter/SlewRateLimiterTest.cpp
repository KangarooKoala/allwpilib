// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>
#include <wpi/timestamp.h>

#include "frc/filter/SlewRateLimiter.h"
#include "frc/units.h"

static mp::quantity<mp::s> now = 0.0 * mp::s;

class SlewRateLimiterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    WPI_SetNowImpl(
        [] { return static_cast<uint64_t>(mp::value(now.in(mp::µs))); });
  }

  void TearDown() override { WPI_SetNowImpl(nullptr); }
};

TEST_F(SlewRateLimiterTest, SlewRateLimit) {
  // WPI_SetNowImpl([] { return
  // static_cast<uint64_t>(mp::value(now.in(mp::µs))); });

  frc::SlewRateLimiter<mp::m> limiter(1.0 * mp::m / mp::s);

  now += 1.0 * mp::s;

  EXPECT_LT(limiter.Calculate(2.0 * mp::m), 2.0 * mp::m);
}

TEST_F(SlewRateLimiterTest, SlewRateNoLimit) {
  frc::SlewRateLimiter<mp::m> limiter(1.0 * mp::m / mp::s);

  now += 1.0 * mp::s;

  EXPECT_EQ(limiter.Calculate(0.5 * mp::m), 0.5 * mp::m);
}

TEST_F(SlewRateLimiterTest, SlewRatePositiveNegativeLimit) {
  frc::SlewRateLimiter<mp::m> limiter(1.0 * mp::m / mp::s,
                                      -0.5 * mp::m / mp::s);

  now += 1.0 * mp::s;

  EXPECT_EQ(limiter.Calculate(2.0 * mp::m), 1.0 * mp::m);

  now += 1.0 * mp::s;

  EXPECT_EQ(limiter.Calculate(0.0 * mp::m), 0.5 * mp::m);
}
