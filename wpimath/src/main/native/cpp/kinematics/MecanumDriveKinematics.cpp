// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/kinematics/MecanumDriveKinematics.h"

#include "frc/units.h"

using namespace frc;

MecanumDriveWheelSpeeds MecanumDriveKinematics::ToWheelSpeeds(
    const ChassisSpeeds& chassisSpeeds,
    const Translation2d& centerOfRotation) const {
  // We have a new center of rotation. We need to compute the matrix again.
  if (centerOfRotation != m_previousCoR) {
    auto fl = m_frontLeftWheel - centerOfRotation;
    auto fr = m_frontRightWheel - centerOfRotation;
    auto rl = m_rearLeftWheel - centerOfRotation;
    auto rr = m_rearRightWheel - centerOfRotation;

    SetInverseKinematics(fl, fr, rl, rr);

    m_previousCoR = centerOfRotation;
  }

  Eigen::Vector3d chassisSpeedsVector{mp::value(chassisSpeeds.vx),
                                      mp::value(chassisSpeeds.vy),
                                      mp::value(chassisSpeeds.omega)};

  Eigen::Vector4d wheelsVector = m_inverseKinematics * chassisSpeedsVector;

  MecanumDriveWheelSpeeds wheelSpeeds;
  wheelSpeeds.frontLeft = wheelsVector(0) * mp::m / mp::s;
  wheelSpeeds.frontRight = wheelsVector(1) * mp::m / mp::s;
  wheelSpeeds.rearLeft = wheelsVector(2) * mp::m / mp::s;
  wheelSpeeds.rearRight = wheelsVector(3) * mp::m / mp::s;
  return wheelSpeeds;
}

ChassisSpeeds MecanumDriveKinematics::ToChassisSpeeds(
    const MecanumDriveWheelSpeeds& wheelSpeeds) const {
  Eigen::Vector4d wheelSpeedsVector{
      mp::value(wheelSpeeds.frontLeft), mp::value(wheelSpeeds.frontRight),
      mp::value(wheelSpeeds.rearLeft), mp::value(wheelSpeeds.rearRight)};

  Eigen::Vector3d chassisSpeedsVector =
      m_forwardKinematics.solve(wheelSpeedsVector);

  return {chassisSpeedsVector(0) * mp::m / mp::s,
          chassisSpeedsVector(1) * mp::m / mp::s,
          chassisSpeedsVector(2) * mp::rad / mp::s};
}

Twist2d MecanumDriveKinematics::ToTwist2d(
    const MecanumDriveWheelPositions& start,
    const MecanumDriveWheelPositions& end) const {
  Eigen::Vector4d wheelDeltasVector{
      mp::value(end.frontLeft) - mp::value(start.frontLeft),
      mp::value(end.frontRight) - mp::value(start.frontRight),
      mp::value(end.rearLeft) - mp::value(start.rearLeft),
      mp::value(end.rearRight) - mp::value(start.rearRight)};

  Eigen::Vector3d twistVector = m_forwardKinematics.solve(wheelDeltasVector);

  return {twistVector(0) * mp::m, twistVector(1) * mp::m,
          twistVector(2) * mp::rad};
}

Twist2d MecanumDriveKinematics::ToTwist2d(
    const MecanumDriveWheelPositions& wheelDeltas) const {
  Eigen::Vector4d wheelDeltasVector{
      mp::value(wheelDeltas.frontLeft), mp::value(wheelDeltas.frontRight),
      mp::value(wheelDeltas.rearLeft), mp::value(wheelDeltas.rearRight)};

  Eigen::Vector3d twistVector = m_forwardKinematics.solve(wheelDeltasVector);

  return {twistVector(0) * mp::m, twistVector(1) * mp::m,
          twistVector(2) * mp::rad};
}

void MecanumDriveKinematics::SetInverseKinematics(Translation2d fl,
                                                  Translation2d fr,
                                                  Translation2d rl,
                                                  Translation2d rr) const {
  m_inverseKinematics = Matrixd<4, 3>{{1, -1, mp::value(-(fl.X() + fl.Y()))},
                                      {1, 1, mp::value(fr.X() - fr.Y())},
                                      {1, 1, mp::value(rl.X() - rl.Y())},
                                      {1, -1, mp::value(-(rr.X() + rr.Y()))}};
}
