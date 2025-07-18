// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "../../ProtoTestBase.h"
#include "frc/controller/DifferentialDriveFeedforward.h"

using namespace frc;

struct DifferentialDriveFeedforwardProtoTestData {
  using Type = DifferentialDriveFeedforward;

  inline static const Type kTestData{
      0.174 * mp::V / (mp::m / mp::s), 0.229 * mp::V / (mp::m / mp::s2),
      4.4 * mp::V / (mp::m / mp::s), 4.5 * mp::V / (mp::m / mp::s2)};

  static void CheckEq(const Type& testData, const Type& data) {
    EXPECT_EQ(mp::value(testData.m_kVLinear), mp::value(data.m_kVLinear));
    EXPECT_EQ(mp::value(testData.m_kALinear), mp::value(data.m_kALinear));
    EXPECT_EQ(mp::value(testData.m_kVAngular), mp::value(data.m_kVAngular));
    EXPECT_EQ(mp::value(testData.m_kAAngular), mp::value(data.m_kAAngular));
  }
};

INSTANTIATE_TYPED_TEST_SUITE_P(DifferentialDriveFeedforward, ProtoTest,
                               DifferentialDriveFeedforwardProtoTestData);
