// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/math/filter/SlewRateLimiter.hpp"

#include <gtest/gtest.h>
#include <wpi/units/length.h>
#include <wpi/units/time.h>
#include <wpi/units/velocity.h>

#include "wpi/util/timestamp.h"

static wpi::units::seconds<> now = 0_s;

class SlewRateLimiterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    WPI_SetNowImpl(
        [] { return wpi::units::microseconds<>{now}.to<uint64_t>(); });
  }

  void TearDown() override { WPI_SetNowImpl(nullptr); }
};

TEST_F(SlewRateLimiterTest, SlewRateLimit) {
  WPI_SetNowImpl([] { return wpi::units::microseconds<>{now}.to<uint64_t>(); });

  wpi::math::SlewRateLimiter<wpi::units::meters_> limiter(1_mps);

  now += 1_s;

  EXPECT_LT(limiter.Calculate(2_m), 2_m);
}

TEST_F(SlewRateLimiterTest, SlewRateNoLimit) {
  wpi::math::SlewRateLimiter<wpi::units::meters_> limiter(1_mps);

  now += 1_s;

  EXPECT_EQ(limiter.Calculate(0.5_m), 0.5_m);
}

TEST_F(SlewRateLimiterTest, SlewRatePositiveNegativeLimit) {
  wpi::math::SlewRateLimiter<wpi::units::meters_> limiter(1_mps, -0.5_mps);

  now += 1_s;

  EXPECT_EQ(limiter.Calculate(2_m), 1_m);

  now += 1_s;

  EXPECT_EQ(limiter.Calculate(0_m), 0.5_m);
}
