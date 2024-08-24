// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

#include <frc/RobotController.h>

void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
  auto leftFeedforward = m_feedforward.Calculate(speeds.left);
  auto rightFeedforward = m_feedforward.Calculate(speeds.right);
  double leftOutput = m_leftPIDController.Calculate(m_leftEncoder.GetRate(),
                                                    speeds.left.value());
  double rightOutput = m_rightPIDController.Calculate(m_rightEncoder.GetRate(),
                                                      speeds.right.value());

  m_leftLeader.SetVoltage(leftOutput * units::volt + leftFeedforward);
  m_rightLeader.SetVoltage(rightOutput * units::volt + rightFeedforward);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot) {
  SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetRotation2d(),
                    m_leftEncoder.GetDistance() * units::meter,
                    m_rightEncoder.GetDistance() * units::meter);
}

void Drivetrain::ResetOdometry(const frc::Pose2d& pose) {
  m_drivetrainSimulator.SetPose(pose);
  m_odometry.ResetPosition(m_gyro.GetRotation2d(),
                           m_leftEncoder.GetDistance() * units::meter,
                           m_rightEncoder.GetDistance() * units::meter, pose);
}

void Drivetrain::SimulationPeriodic() {
  // To update our simulation, we set motor voltage inputs, update the
  // simulation, and write the simulated positions and velocities to our
  // simulated encoder and gyro. We negate the right side so that positive
  // voltages make the right side move forward.
  m_drivetrainSimulator.SetInputs(m_leftLeader.Get() * units::volt *
                                      frc::RobotController::GetInputVoltage(),
                                  m_rightLeader.Get() * units::volt *
                                      frc::RobotController::GetInputVoltage());
  m_drivetrainSimulator.Update(20_ms);

  m_leftEncoderSim.SetDistance(m_drivetrainSimulator.GetLeftPosition().value());
  m_leftEncoderSim.SetRate(m_drivetrainSimulator.GetLeftVelocity().value());
  m_rightEncoderSim.SetDistance(
      m_drivetrainSimulator.GetRightPosition().value());
  m_rightEncoderSim.SetRate(m_drivetrainSimulator.GetRightVelocity().value());
  m_gyroSim.SetAngle(-m_drivetrainSimulator.GetHeading().Degrees().value());
}

void Drivetrain::Periodic() {
  UpdateOdometry();
  m_fieldSim.SetRobotPose(m_odometry.GetPose());
}
