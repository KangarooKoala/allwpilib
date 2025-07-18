// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "../../StructTestBase.h"
#include "frc/controller/SimpleMotorFeedforward.h"
#include "frc/controller/struct/SimpleMotorFeedforwardStruct.h"
#include "frc/units.h"

using namespace frc;

template <mp::Unit auto Distance>
struct SimpleMotorFeedforwardStructTestData {
  using Type = SimpleMotorFeedforward<Distance>;

  inline static const Type kTestData = {
      0.4 * mp::V, 4.0 * mp::V / (Distance / mp::s),
      0.7 * mp::V / (Distance / mp::s2), 25.0 * mp::ms};

  static void CheckEq(const Type& testData, const Type& data) {
    EXPECT_EQ(mp::value(testData.GetKs()), mp::value(data.GetKs()));
    EXPECT_EQ(mp::value(testData.GetKv()), mp::value(data.GetKv()));
    EXPECT_EQ(mp::value(testData.GetKa()), mp::value(data.GetKa()));
    EXPECT_EQ(mp::value(testData.GetDt()), mp::value(data.GetDt()));
  }
};

INSTANTIATE_TYPED_TEST_SUITE_P(SimpleMotorFeedforwardMeters, StructTest,
                               SimpleMotorFeedforwardStructTestData<mp::m>);
INSTANTIATE_TYPED_TEST_SUITE_P(SimpleMotorFeedforwardFeet, StructTest,
                               SimpleMotorFeedforwardStructTestData<mp::cm>);
INSTANTIATE_TYPED_TEST_SUITE_P(SimpleMotorFeedforwardRadians, StructTest,
                               SimpleMotorFeedforwardStructTestData<mp::rad>);
