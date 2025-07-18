// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include <gtest/gtest.h>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/interpolation/TimeInterpolatableBuffer.h"
#include "frc/units.h"

TEST(TimeInterpolatableBufferTest, AddSample) {
  frc::TimeInterpolatableBuffer<frc::Rotation2d> buffer{10.0 * mp::s};

  // No entries
  buffer.AddSample(1.0 * mp::s, 0.0 * mp::rad);
  EXPECT_TRUE(buffer.Sample(1.0 * mp::s).value() == 0.0 * mp::rad);

  // New entry at start of container
  buffer.AddSample(0.0 * mp::s, 1.0 * mp::rad);
  EXPECT_TRUE(buffer.Sample(0.0 * mp::s).value() == 1.0 * mp::rad);

  // New entry in middle of container
  buffer.AddSample(0.5 * mp::s, 0.5 * mp::rad);
  EXPECT_TRUE(buffer.Sample(0.5 * mp::s).value() == 0.5 * mp::rad);

  // Override sample
  buffer.AddSample(0.5 * mp::s, 1.0 * mp::rad);
  EXPECT_TRUE(buffer.Sample(0.5 * mp::s).value() == 1.0 * mp::rad);
}

TEST(TimeInterpolatableBufferTest, Interpolation) {
  frc::TimeInterpolatableBuffer<frc::Rotation2d> buffer{10.0 * mp::s};

  buffer.AddSample(0.0 * mp::s, 0.0 * mp::rad);
  EXPECT_TRUE(buffer.Sample(0.0 * mp::s).value() == 0.0 * mp::rad);
  buffer.AddSample(1.0 * mp::s, 1.0 * mp::rad);
  EXPECT_TRUE(buffer.Sample(0.5 * mp::s).value() == 0.5 * mp::rad);
  EXPECT_TRUE(buffer.Sample(1.0 * mp::s).value() == 1.0 * mp::rad);
  buffer.AddSample(3.0 * mp::s, 2.0 * mp::rad);
  EXPECT_TRUE(buffer.Sample(2.0 * mp::s).value() == 1.5 * mp::rad);

  buffer.AddSample(10.5 * mp::s, 2.0 * mp::rad);
  EXPECT_TRUE(buffer.Sample(0.0 * mp::s) == 1.0 * mp::rad);
}

TEST(TimeInterpolatableBufferTest, Pose2d) {
  frc::TimeInterpolatableBuffer<frc::Pose2d> buffer{10.0 * mp::s};

  // We expect to be at (1 - 1/std::sqrt(2), 1/std::sqrt(2), 45deg) at t=0.5
  buffer.AddSample(0.0 * mp::s,
                   frc::Pose2d{0.0 * mp::m, 0.0 * mp::m, 90.0 * mp::deg});
  buffer.AddSample(1.0 * mp::s,
                   frc::Pose2d{1.0 * mp::m, 1.0 * mp::m, 0.0 * mp::deg});
  frc::Pose2d sample = buffer.Sample(0.5 * mp::s).value();

  EXPECT_TRUE(std::abs(mp::value(sample.X()) - (1.0 - 1.0 / std::sqrt(2.0))) <
              0.01);
  EXPECT_TRUE(std::abs(mp::value(sample.Y()) - (1.0 / std::sqrt(2.0))) < 0.01);
  EXPECT_TRUE(std::abs(mp::value(sample.Rotation().Degrees()) - 45.0) < 0.01);
}
