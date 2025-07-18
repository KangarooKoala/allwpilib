// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "frc/controller/DifferentialDriveFeedforward.h"
#include "frc/controller/LinearPlantInversionFeedforward.h"
#include "frc/system/plant/LinearSystemId.h"
#include "frc/units.h"

TEST(DifferentialDriveFeedforwardTest, CalculateWithTrackwidth) {
  constexpr auto kVLinear = 1.0 * mp::V / (mp::m / mp::s);
  constexpr auto kALinear = 1.0 * mp::V / (mp::m / mp::s2);
  constexpr auto kVAngular = 1.0 * mp::V / (mp::rad / mp::s);
  constexpr auto kAAngular = 1.0 * mp::V / (mp::rad / mp::s2);
  constexpr auto trackwidth = 1.0 * mp::m;
  constexpr auto dt = 20.0 * mp::ms;

  frc::DifferentialDriveFeedforward differentialDriveFeedforward{
      kVLinear, kALinear, kVAngular, kAAngular, trackwidth};
  frc::LinearSystem<2, 2, 2> plant =
      frc::LinearSystemId::IdentifyDrivetrainSystem(
          kVLinear, kALinear, kVAngular, kAAngular, trackwidth);
  for (auto currentLeftVelocity = -4.0 * mp::m / mp::s;
       currentLeftVelocity <= 4.0 * mp::m / mp::s;
       currentLeftVelocity += 2.0 * mp::m / mp::s) {
    for (auto currentRightVelocity = -4.0 * mp::m / mp::s;
         currentRightVelocity <= 4.0 * mp::m / mp::s;
         currentRightVelocity += 2.0 * mp::m / mp::s) {
      for (auto nextLeftVelocity = -4.0 * mp::m / mp::s;
           nextLeftVelocity <= 4.0 * mp::m / mp::s;
           nextLeftVelocity += 2.0 * mp::m / mp::s) {
        for (auto nextRightVelocity = -4.0 * mp::m / mp::s;
             nextRightVelocity <= 4.0 * mp::m / mp::s;
             nextRightVelocity += 2.0 * mp::m / mp::s) {
          auto [left, right] = differentialDriveFeedforward.Calculate(
              currentLeftVelocity, nextLeftVelocity, currentRightVelocity,
              nextRightVelocity, dt);
          Eigen::Vector2d nextX = plant.CalculateX(
              Eigen::Vector2d{mp::value(currentLeftVelocity),
                              mp::value(currentRightVelocity)},
              Eigen::Vector2d{mp::value(left), mp::value(right)}, dt);
          EXPECT_NEAR(nextX(0), mp::value(nextLeftVelocity), 1e-6);
          EXPECT_NEAR(nextX(1), mp::value(nextRightVelocity), 1e-6);
        }
      }
    }
  }
}

TEST(DifferentialDriveFeedforwardTest, CalculateWithoutTrackwidth) {
  constexpr auto kVLinear = 1.0 * mp::V / (mp::m / mp::s);
  constexpr auto kALinear = 1.0 * mp::V / (mp::m / mp::s2);
  constexpr auto kVAngular = 1.0 * mp::V / (mp::m / mp::s);
  constexpr auto kAAngular = 1.0 * mp::V / (mp::m / mp::s2);
  constexpr auto dt = 20.0 * mp::ms;

  frc::DifferentialDriveFeedforward differentialDriveFeedforward{
      kVLinear, kALinear, kVAngular, kAAngular};
  frc::LinearSystem<2, 2, 2> plant =
      frc::LinearSystemId::IdentifyDrivetrainSystem(kVLinear, kALinear,
                                                    kVAngular, kAAngular);
  for (auto currentLeftVelocity = -4.0 * mp::m / mp::s;
       currentLeftVelocity <= 4.0 * mp::m / mp::s;
       currentLeftVelocity += 2.0 * mp::m / mp::s) {
    for (auto currentRightVelocity = -4.0 * mp::m / mp::s;
         currentRightVelocity <= 4.0 * mp::m / mp::s;
         currentRightVelocity += 2.0 * mp::m / mp::s) {
      for (auto nextLeftVelocity = -4.0 * mp::m / mp::s;
           nextLeftVelocity <= 4.0 * mp::m / mp::s;
           nextLeftVelocity += 2.0 * mp::m / mp::s) {
        for (auto nextRightVelocity = -4.0 * mp::m / mp::s;
             nextRightVelocity <= 4.0 * mp::m / mp::s;
             nextRightVelocity += 2.0 * mp::m / mp::s) {
          auto [left, right] = differentialDriveFeedforward.Calculate(
              currentLeftVelocity, nextLeftVelocity, currentRightVelocity,
              nextRightVelocity, dt);
          Eigen::Vector2d nextX = plant.CalculateX(
              Eigen::Vector2d{mp::value(currentLeftVelocity),
                              mp::value(currentRightVelocity)},
              Eigen::Vector2d{mp::value(left), mp::value(right)}, dt);
          EXPECT_NEAR(nextX(0), mp::value(nextLeftVelocity), 1e-6);
          EXPECT_NEAR(nextX(1), mp::value(nextRightVelocity), 1e-6);
        }
      }
    }
  }
}
