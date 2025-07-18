// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>
#include <wpi/timestamp.h>

#include "frc/filter/Debouncer.h"
#include "frc/units.h"

static mp::quantity<mp::s> now = 0.0 * mp::s;

class DebouncerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    WPI_SetNowImpl(
        [] { return static_cast<uint64_t>(mp::value(now.in(mp::µs))); });
  }

  void TearDown() override { WPI_SetNowImpl(nullptr); }
};

TEST_F(DebouncerTest, DebounceRising) {
  frc::Debouncer debouncer{20.0 * mp::ms};

  debouncer.Calculate(false);
  EXPECT_FALSE(debouncer.Calculate(true));

  now += 1.0 * mp::s;

  EXPECT_TRUE(debouncer.Calculate(true));
}

TEST_F(DebouncerTest, DebounceFalling) {
  frc::Debouncer debouncer{20.0 * mp::ms,
                           frc::Debouncer::DebounceType::kFalling};

  debouncer.Calculate(true);
  EXPECT_TRUE(debouncer.Calculate(false));

  now += 1.0 * mp::s;

  EXPECT_FALSE(debouncer.Calculate(false));
}

TEST_F(DebouncerTest, DebounceBoth) {
  frc::Debouncer debouncer{20.0 * mp::ms, frc::Debouncer::DebounceType::kBoth};

  debouncer.Calculate(false);
  EXPECT_FALSE(debouncer.Calculate(true));

  now += 1.0 * mp::s;

  EXPECT_TRUE(debouncer.Calculate(true));
  EXPECT_TRUE(debouncer.Calculate(false));

  now += 1.0 * mp::s;

  EXPECT_FALSE(debouncer.Calculate(false));
}

TEST_F(DebouncerTest, DebounceParams) {
  frc::Debouncer debouncer{20.0 * mp::ms, frc::Debouncer::DebounceType::kBoth};

  EXPECT_TRUE(debouncer.GetDebounceTime() == 20.0 * mp::ms);
  EXPECT_TRUE(debouncer.GetDebounceType() ==
              frc::Debouncer::DebounceType::kBoth);

  debouncer.SetDebounceTime(100.0 * mp::ms);

  EXPECT_TRUE(debouncer.GetDebounceTime() == 100.0 * mp::ms);

  debouncer.SetDebounceType(frc::Debouncer::DebounceType::kFalling);

  EXPECT_TRUE(debouncer.GetDebounceType() ==
              frc::Debouncer::DebounceType::kFalling);

  EXPECT_TRUE(debouncer.Calculate(false));
}
