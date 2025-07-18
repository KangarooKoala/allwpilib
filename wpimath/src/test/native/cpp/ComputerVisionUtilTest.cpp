// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include "frc/ComputerVisionUtil.h"
#include "frc/units.h"

TEST(ComputerVisionUtilTest, ObjectToRobotPose) {
  frc::Pose3d robot{
      1.0 * mp::m, 2.0 * mp::m, 0.0 * mp::m,
      frc::Rotation3d{0.0 * mp::deg, 0.0 * mp::deg, 30.0 * mp::deg}};
  frc::Transform3d cameraToObject{
      frc::Translation3d{1.0 * mp::m, 1.0 * mp::m, 1.0 * mp::m},
      frc::Rotation3d{0.0 * mp::deg, -20.0 * mp::deg, 45.0 * mp::deg}};
  frc::Transform3d robotToCamera{
      frc::Translation3d{1.0 * mp::m, 0.0 * mp::m, 2.0 * mp::m},
      frc::Rotation3d{0.0 * mp::deg, 0.0 * mp::deg, 25.0 * mp::deg}};
  frc::Pose3d object = robot + robotToCamera + cameraToObject;

  EXPECT_EQ(robot,
            frc::ObjectToRobotPose(object, cameraToObject, robotToCamera));
}
