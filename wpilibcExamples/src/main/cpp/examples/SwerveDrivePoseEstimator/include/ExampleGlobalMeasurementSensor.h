// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/StateSpaceUtil.h>
#include <frc/geometry/Pose2d.h>

/**
 * This dummy class represents a global measurement sensor, such as a computer
 * vision solution.
 */
class ExampleGlobalMeasurementSensor {
 public:
  static frc::Pose2d GetEstimatedGlobalPose(
      const frc::Pose2d& estimatedRobotPose) {
    auto randVec = frc::MakeWhiteNoiseVector(0.1, 0.1, 0.1);
    return frc::Pose2d{estimatedRobotPose.X() + randVec(0) * units::meter,
                       estimatedRobotPose.Y() + randVec(1) * units::meter,
                       estimatedRobotPose.Rotation() +
                           frc::Rotation2d{randVec(2) * units::radian}};
  }
};
