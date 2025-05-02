// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/controller/LTVDifferentialDriveController.h"

#include <cmath>

#include "frc/DARE.h"
#include "frc/MathUtil.h"
#include "frc/system/Discretization.h"
#include "frc/units.h"

using namespace frc;

DifferentialDriveWheelVoltages LTVDifferentialDriveController::Calculate(
    const Pose2d& currentPose, mp::quantity<mp::m / mp::s> leftVelocity,
    mp::quantity<mp::m / mp::s> rightVelocity, const Pose2d& poseRef,
    mp::quantity<mp::m / mp::s> leftVelocityRef,
    mp::quantity<mp::m / mp::s> rightVelocityRef) {
  // This implements the linear time-varying differential drive controller in
  // theorem 8.7.4 of https://controls-in-frc.link/
  //
  //     [x ]
  //     [y ]       [Vₗ]
  // x = [θ ]   u = [Vᵣ]
  //     [vₗ]
  //     [vᵣ]

  mp::quantity<mp::m / mp::s> velocity = (leftVelocity + rightVelocity) / 2.0;

  // The DARE is ill-conditioned if the velocity is close to zero, so don't
  // let the system stop.
  if (mp::abs(velocity) < 1e-4 * mp::m / mp::s) {
    velocity = 1e-4 * mp::m / mp::s;
  }

  Eigen::Vector<double, 5> r{mp::value(poseRef.X()), mp::value(poseRef.Y()),
                             mp::value(poseRef.Rotation().Radians()),
                             mp::value(leftVelocityRef),
                             mp::value(rightVelocityRef)};
  Eigen::Vector<double, 5> x{mp::value(currentPose.X()),
                             mp::value(currentPose.Y()),
                             mp::value(currentPose.Rotation().Radians()),
                             mp::value(leftVelocity), mp::value(rightVelocity)};

  m_error = r - x;
  m_error(2) = mp::value(frc::AngleModulus(m_error(2) * mp::rad));

  Eigen::Matrix<double, 5, 5> A{{0.0, 0.0, 0.0, 0.5, 0.5},
                                {0.0, 0.0, mp::value(velocity), 0.0, 0.0},
                                {0.0, 0.0, 0.0, -1.0 / mp::value(m_trackwidth),
                                 1.0 / mp::value(m_trackwidth)},
                                {0.0, 0.0, 0.0, m_A(0, 0), m_A(0, 1)},
                                {0.0, 0.0, 0.0, m_A(1, 0), m_A(1, 1)}};
  Eigen::Matrix<double, 5, 2> B{{0.0, 0.0},
                                {0.0, 0.0},
                                {0.0, 0.0},
                                {m_B(0, 0), m_B(0, 1)},
                                {m_B(1, 0), m_B(1, 1)}};

  Eigen::Matrix<double, 5, 5> discA;
  Eigen::Matrix<double, 5, 2> discB;
  DiscretizeAB(A, B, m_dt, &discA, &discB);

  auto S = DARE<5, 2>(discA, discB, m_Q, m_R, false).value();

  // K = (BᵀSB + R)⁻¹BᵀSA
  Eigen::Matrix<double, 2, 5> K = (discB.transpose() * S * discB + m_R)
                                      .llt()
                                      .solve(discB.transpose() * S * discA);

  Eigen::Matrix<double, 5, 5> inRobotFrame{
      {std::cos(x(2)), std::sin(x(2)), 0.0, 0.0, 0.0},
      {-std::sin(x(2)), std::cos(x(2)), 0.0, 0.0, 0.0},
      {0.0, 0.0, 1.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 1.0}};

  Eigen::Vector2d u = K * inRobotFrame * m_error;

  return DifferentialDriveWheelVoltages{u(0) * mp::V, u(1) * mp::V};
}
