// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "../../StructTestBase.h"
#include "frc/controller/SimpleMotorFeedforward.h"
#include "frc/controller/struct/SimpleMotorFeedforwardStruct.h"
#include "units/acceleration.h"
#include "units/velocity.h"

using namespace frc;

struct SimpleMotorFeedforwardStructTestData {
  using Type = SimpleMotorFeedforward<units::meters>;

  inline static const Type kTestData = {0.4_V,
                                        4.0_V / 1_mps,
                                        0.7_V / 1_mps_sq, 25_ms};

  static void CheckEq(const Type& testData, const Type& data) {
    EXPECT_EQ(testData.GetKs().value(), data.GetKs().value());
    EXPECT_EQ(testData.GetKv().value(), data.GetKv().value());
    EXPECT_EQ(testData.GetKa().value(), data.GetKa().value());
    EXPECT_EQ(testData.GetDt().value(), data.GetDt().value());
  }
};

INSTANTIATE_TYPED_TEST_SUITE_P(SimpleMotorFeedforwardMeters, StructTest,
                               SimpleMotorFeedforwardStructTestData);
