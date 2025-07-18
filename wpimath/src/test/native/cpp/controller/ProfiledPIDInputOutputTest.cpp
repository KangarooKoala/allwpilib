// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <gtest/gtest.h>

#include "frc/controller/ProfiledPIDController.h"
#include "frc/units.h"

TEST(ProfiledPIDInputOutputTest, ContinuousInput1) {
  frc::ProfiledPIDController<mp::deg> controller{
      0.0, 0.0, 0.0, {360.0 * mp::deg / mp::s, 180.0 * mp::deg / mp::s2}};

  controller.SetP(1);
  controller.EnableContinuousInput(-180.0 * mp::deg, 180.0 * mp::deg);

  static constexpr mp::quantity<mp::deg> kSetpoint = -179.0 * mp::deg;
  static constexpr mp::quantity<mp::deg> kMeasurement = -179.0 * mp::deg;
  static constexpr mp::quantity<mp::deg> kGoal = 179.0 * mp::deg;

  controller.Reset(kSetpoint);
  EXPECT_LT(controller.Calculate(kMeasurement, kGoal), 0.0);

  // Error must be less than half the input range at all times
  EXPECT_LT(mp::abs(controller.GetSetpoint().position - kMeasurement),
            180.0 * mp::deg);
}

TEST(ProfiledPIDInputOutputTest, ContinuousInput2) {
  frc::ProfiledPIDController<mp::deg> controller{
      0.0, 0.0, 0.0, {360.0 * mp::deg / mp::s, 180.0 * mp::deg / mp::s2}};

  controller.SetP(1);
  controller.EnableContinuousInput(-std::numbers::pi * mp::rad,
                                   std::numbers::pi * mp::rad);

  static constexpr mp::quantity<mp::rad> kSetpoint =
      -3.4826633343199735 * mp::rad;
  static constexpr mp::quantity<mp::rad> kMeasurement =
      -3.1352207333939606 * mp::rad;
  static constexpr mp::quantity<mp::rad> kGoal = -3.534162788601621 * mp::rad;

  controller.Reset(kSetpoint);
  EXPECT_LT(controller.Calculate(kMeasurement, kGoal), 0.0);

  // Error must be less than half the input range at all times
  EXPECT_LT(mp::abs(controller.GetSetpoint().position - kMeasurement),
            std::numbers::pi * mp::rad);
}

TEST(ProfiledPIDInputOutputTest, ContinuousInput3) {
  frc::ProfiledPIDController<mp::deg> controller{
      0.0, 0.0, 0.0, {360.0 * mp::deg / mp::s, 180.0 * mp::deg / mp::s2}};

  controller.SetP(1);
  controller.EnableContinuousInput(-std::numbers::pi * mp::rad,
                                   std::numbers::pi * mp::rad);

  static constexpr mp::quantity<mp::rad> kSetpoint =
      -3.5176604690006377 * mp::rad;
  static constexpr mp::quantity<mp::rad> kMeasurement =
      3.1191729343822456 * mp::rad;
  static constexpr mp::quantity<mp::rad> kGoal = 2.709680418117445 * mp::rad;

  controller.Reset(kSetpoint);
  EXPECT_LT(controller.Calculate(kMeasurement, kGoal), 0.0);

  // Error must be less than half the input range at all times
  EXPECT_LT(mp::abs(controller.GetSetpoint().position - kMeasurement),
            std::numbers::pi * mp::rad);
}

TEST(ProfiledPIDInputOutputTest, ContinuousInput4) {
  frc::ProfiledPIDController<mp::deg> controller{
      0.0, 0.0, 0.0, {360.0 * mp::deg / mp::s, 180.0 * mp::deg / mp::s2}};

  controller.SetP(1);
  controller.EnableContinuousInput(0.0 * mp::rad,
                                   2.0 * std::numbers::pi * mp::rad);

  static constexpr mp::quantity<mp::rad> kSetpoint = 2.78 * mp::rad;
  static constexpr mp::quantity<mp::rad> kMeasurement = 3.12 * mp::rad;
  static constexpr mp::quantity<mp::rad> kGoal = 2.71 * mp::rad;

  controller.Reset(kSetpoint);
  EXPECT_LT(controller.Calculate(kMeasurement, kGoal), 0.0);

  // Error must be less than half the input range at all times
  EXPECT_LT(mp::abs(controller.GetSetpoint().position - kMeasurement),
            std::numbers::pi * mp::rad);
}

TEST(ProfiledPIDInputOutputTest, ProportionalGainOutput) {
  frc::ProfiledPIDController<mp::deg> controller{
      0.0, 0.0, 0.0, {360.0 * mp::deg / mp::s, 180.0 * mp::deg / mp::s2}};

  controller.SetP(4);

  EXPECT_DOUBLE_EQ(-0.1, controller.Calculate(0.025 * mp::deg, 0.0 * mp::deg));
}

TEST(ProfiledPIDInputOutputTest, IntegralGainOutput) {
  frc::ProfiledPIDController<mp::deg> controller{
      0.0, 0.0, 0.0, {360.0 * mp::deg / mp::s, 180.0 * mp::deg / mp::s2}};

  controller.SetI(4);

  double out = 0;

  for (int i = 0; i < 5; i++) {
    out = controller.Calculate(0.025 * mp::deg, 0.0 * mp::deg);
  }

  EXPECT_DOUBLE_EQ(-0.5 * mp::value(controller.GetPeriod()), out);
}

TEST(ProfiledPIDInputOutputTest, DerivativeGainOutput) {
  frc::ProfiledPIDController<mp::deg> controller{
      0.0, 0.0, 0.0, {360.0 * mp::deg / mp::s, 180.0 * mp::deg / mp::s2}};

  controller.SetD(4);

  controller.Calculate(0.0 * mp::deg, 0.0 * mp::deg);

  EXPECT_DOUBLE_EQ(-0.01 / mp::value(controller.GetPeriod()),
                   controller.Calculate(0.0025 * mp::deg, 0.0 * mp::deg));
}
