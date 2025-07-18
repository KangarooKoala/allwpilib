// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cassert>
#include <span>
#include <vector>

#include <gtest/gtest.h>
#include <wpi/array.h>
#include <wpi/circular_buffer.h>

#include "frc/EigenCore.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/path/TravelingSalesman.h"
#include "frc/units.h"

/**
 * Returns true if the cycles represented by the two lists match.
 *
 * @param expected The expected cycle.
 * @param actual The actual cycle.
 */
bool IsMatchingCycle(std::span<const frc::Pose2d> expected,
                     std::span<const frc::Pose2d> actual) {
  assert(expected.size() == actual.size());

  // Check actual has expected cycle (forward)
  wpi::circular_buffer<frc::Pose2d> actualBufferForward{expected.size()};
  for (size_t i = 0; i < actual.size(); ++i) {
    actualBufferForward.push_back(actual[i % actual.size()]);
  }
  bool matchesExpectedForward = true;
  for (size_t i = 0; i < expected.size(); ++i) {
    matchesExpectedForward &= (expected[i] == actualBufferForward[i]);
  }

  // Check actual has expected cycle (reverse)
  wpi::circular_buffer<frc::Pose2d> actualBufferReverse{expected.size()};
  for (size_t i = 0; i < actual.size(); ++i) {
    actualBufferReverse.push_front(actual[(1 + i) % actual.size()]);
  }
  bool matchesExpectedReverse = true;
  for (size_t i = 0; i < expected.size(); ++i) {
    matchesExpectedReverse &= (expected[i] == actualBufferReverse[i]);
  }

  // Actual may be reversed from expected, but that's still valid
  return matchesExpectedForward || matchesExpectedReverse;
}

TEST(TravelingSalesmanTest, FiveLengthStaticPathWithDistanceCost) {
  // ...................
  // ........2..........
  // ..0..........4.....
  // ...................
  // ....3.....1........
  // ...................
  wpi::array<frc::Pose2d, 5> poses{
      frc::Pose2d{3.0 * mp::m, 3.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{11.0 * mp::m, 5.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{9.0 * mp::m, 2.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{5.0 * mp::m, 5.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{14.0 * mp::m, 3.0 * mp::m, 0.0 * mp::rad}};

  frc::TravelingSalesman traveler;
  wpi::array<frc::Pose2d, 5> solution = traveler.Solve(poses, 500);

  wpi::array<frc::Pose2d, 5> expected{poses[0], poses[2], poses[4], poses[1],
                                      poses[3]};

  EXPECT_TRUE(IsMatchingCycle(expected, solution));
}

TEST(TravelingSalesmanTest, FiveLengthDynamicPathWithDistanceCost) {
  // ...................
  // ........2..........
  // ..0..........4.....
  // ...................
  // ....3.....1........
  // ...................
  wpi::array<frc::Pose2d, 5> poses{
      frc::Pose2d{3.0 * mp::m, 3.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{11.0 * mp::m, 5.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{9.0 * mp::m, 2.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{5.0 * mp::m, 5.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{14.0 * mp::m, 3.0 * mp::m, 0.0 * mp::rad}};

  frc::TravelingSalesman traveler;
  std::vector<frc::Pose2d> solution =
      traveler.Solve(std::span<const frc::Pose2d>{poses}, 500);

  ASSERT_EQ(5u, solution.size());
  wpi::array<frc::Pose2d, 5> expected{poses[0], poses[2], poses[4], poses[1],
                                      poses[3]};

  EXPECT_TRUE(IsMatchingCycle(expected, solution));
}

TEST(TravelingSalesmanTest, TenLengthStaticPathWithDistanceCost) {
  // ....6.3..1.2.......
  // ..4................
  // .............9.....
  // .0.................
  // .....7..5...8......
  // ...................
  wpi::array<frc::Pose2d, 10> poses{
      frc::Pose2d{2.0 * mp::m, 4.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{10.0 * mp::m, 1.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{12.0 * mp::m, 1.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{7.0 * mp::m, 1.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{3.0 * mp::m, 2.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{9.0 * mp::m, 5.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{5.0 * mp::m, 1.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{6.0 * mp::m, 5.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{13.0 * mp::m, 5.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{14.0 * mp::m, 3.0 * mp::m, 0.0 * mp::rad}};

  frc::TravelingSalesman traveler;
  wpi::array<frc::Pose2d, 10> solution = traveler.Solve(poses, 500);

  wpi::array<frc::Pose2d, 10> expected{poses[0], poses[4], poses[6], poses[3],
                                       poses[1], poses[2], poses[9], poses[8],
                                       poses[5], poses[7]};

  EXPECT_TRUE(IsMatchingCycle(expected, solution));
}

TEST(TravelingSalesmanTest, TenLengthDynamicPathWithDistanceCost) {
  // ....6.3..1.2.......
  // ..4................
  // .............9.....
  // .0.................
  // .....7..5...8......
  // ...................
  wpi::array<frc::Pose2d, 10> poses{
      frc::Pose2d{2.0 * mp::m, 4.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{10.0 * mp::m, 1.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{12.0 * mp::m, 1.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{7.0 * mp::m, 1.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{3.0 * mp::m, 2.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{9.0 * mp::m, 5.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{5.0 * mp::m, 1.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{6.0 * mp::m, 5.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{13.0 * mp::m, 5.0 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{14.0 * mp::m, 3.0 * mp::m, 0.0 * mp::rad}};

  frc::TravelingSalesman traveler;
  std::vector<frc::Pose2d> solution =
      traveler.Solve(std::span<const frc::Pose2d>{poses}, 500);

  ASSERT_EQ(10u, solution.size());
  wpi::array<frc::Pose2d, 10> expected{poses[0], poses[4], poses[6], poses[3],
                                       poses[1], poses[2], poses[9], poses[8],
                                       poses[5], poses[7]};

  EXPECT_TRUE(IsMatchingCycle(expected, solution));
}
