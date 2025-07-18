// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "frc/trajectory/Trajectory.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/trajectory/constraint/TrajectoryConstraint.h"
#include "frc/units-usc.h"
#include "frc/units.h"

namespace frc {
class TestTrajectory {
 public:
  static Trajectory GetTrajectory(TrajectoryConfig& config) {
    // 2018 cross scale auto waypoints
    const Pose2d sideStart{1.54 * mp::ft, 23.23 * mp::ft, 180.0 * mp::deg};
    const Pose2d crossScale{23.7 * mp::ft, 6.8 * mp::ft, -160.0 * mp::deg};

    config.SetReversed(true);

    auto vector = std::vector<Translation2d>{
        (sideStart + Transform2d{Translation2d{-13.0 * mp::ft, 0.0 * mp::ft},
                                 0.0 * mp::deg})
            .Translation(),
        (sideStart + Transform2d{Translation2d{-19.5 * mp::ft, 5.0 * mp::ft},
                                 -90.0 * mp::deg})
            .Translation()};

    return TrajectoryGenerator::GenerateTrajectory(sideStart, vector,
                                                   crossScale, config);
  }
};

}  // namespace frc
