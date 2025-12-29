// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.hpp"

void Drivetrain::Drive(wpi::units::meters_per_second<> xSpeed,
                       wpi::units::meters_per_second<> ySpeed,
                       wpi::units::radians_per_second<> rot, bool fieldRelative,
                       wpi::units::seconds<> period) {
  wpi::math::ChassisSpeeds chassisSpeeds{xSpeed, ySpeed, rot};
  if (fieldRelative) {
    chassisSpeeds = chassisSpeeds.ToRobotRelative(m_imu.GetRotation2d());
  }
  chassisSpeeds = chassisSpeeds.Discretize(period);

  auto states = m_kinematics.ToSwerveModuleStates(chassisSpeeds);
  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;
  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_imu.GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                     m_backLeft.GetPosition(), m_backRight.GetPosition()});
}
