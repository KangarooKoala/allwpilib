// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/trajectory/TrajectoryConfig.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/units.h"

TEST(TrajectoryConcatenateTest, States) {
  auto t1 = frc::TrajectoryGenerator::GenerateTrajectory(
      {}, {}, {1.0 * mp::m, 1.0 * mp::m, 0.0 * mp::deg},
      {2.0 * mp::m / mp::s, 2.0 * mp::m / mp::s2});
  auto t2 = frc::TrajectoryGenerator::GenerateTrajectory(
      {1.0 * mp::m, 1.0 * mp::m, 0.0 * mp::deg}, {},
      {2.0 * mp::m, 2.0 * mp::m, 45.0 * mp::deg},
      {2.0 * mp::m / mp::s, 2.0 * mp::m / mp::s2});

  auto t = t1 + t2;

  double time = -1.0;
  for (size_t i = 0; i < t.States().size(); ++i) {
    const auto& state = t.States()[i];

    // Make sure that the timestamps are strictly increasing.
    EXPECT_GT(mp::value(state.t), time);
    time = mp::value(state.t);

    // Ensure that the states in t are the same as those in t1 and t2.
    if (i < t1.States().size()) {
      EXPECT_EQ(state, t1.States()[i]);
    } else {
      auto st = t2.States()[i - t1.States().size() + 1];
      st.t += t1.TotalTime();
      EXPECT_EQ(state, st);
    }
  }
}
