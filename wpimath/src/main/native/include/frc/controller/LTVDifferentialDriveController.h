// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>

#include <Eigen/Core>
#include <wpi/SymbolExports.h>
#include <wpi/array.h>

#include "frc/StateSpaceUtil.h"
#include "frc/controller/DifferentialDriveWheelVoltages.h"
#include "frc/geometry/Pose2d.h"
#include "frc/system/LinearSystem.h"
#include "frc/trajectory/Trajectory.h"
#include "frc/units.h"

namespace frc {

/**
 * The linear time-varying differential drive controller has a similar form to
 * the LQR, but the model used to compute the controller gain is the nonlinear
 * differential drive model linearized around the drivetrain's current state. We
 * precompute gains for important places in our state-space, then interpolate
 * between them with a lookup table to save computational resources.
 *
 * This controller has a flat hierarchy with pose and wheel velocity references
 * and voltage outputs. This is different from a unicycle controller's nested
 * hierarchy where the top-level controller has a pose reference and chassis
 * velocity command outputs, and the low-level controller has wheel velocity
 * references and voltage outputs. Flat hierarchies are easier to tune in one
 * shot.
 *
 * See section 8.7 in Controls Engineering in FRC for a derivation of the
 * control law we used shown in theorem 8.7.4.
 */
class WPILIB_DLLEXPORT LTVDifferentialDriveController {
 public:
  /**
   * Constructs a linear time-varying differential drive controller.
   *
   * See
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html#lqr-tuning
   * for how to select the tolerances.
   *
   * @param plant      The differential drive velocity plant.
   * @param trackwidth The distance between the differential drive's left and
   *                   right wheels.
   * @param Qelems     The maximum desired error tolerance for each state.
   * @param Relems     The maximum desired control effort for each input.
   * @param dt         Discretization timestep.
   */
  LTVDifferentialDriveController(const frc::LinearSystem<2, 2, 2>& plant,
                                 mp::quantity<mp::m> trackwidth,
                                 const wpi::array<double, 5>& Qelems,
                                 const wpi::array<double, 2>& Relems,
                                 mp::quantity<mp::s> dt)
      : m_trackwidth{trackwidth},
        m_A{plant.A()},
        m_B{plant.B()},
        m_Q{frc::MakeCostMatrix(Qelems)},
        m_R{frc::MakeCostMatrix(Relems)},
        m_dt{dt} {}

  /**
   * Move constructor.
   */
  LTVDifferentialDriveController(LTVDifferentialDriveController&&) = default;

  /**
   * Move assignment operator.
   */
  LTVDifferentialDriveController& operator=(LTVDifferentialDriveController&&) =
      default;

  /**
   * Returns true if the pose error is within tolerance of the reference.
   */
  bool AtReference() const {
    return std::abs(m_error(0)) < m_tolerance(0) &&
           std::abs(m_error(1)) < m_tolerance(1) &&
           std::abs(m_error(2)) < m_tolerance(2) &&
           std::abs(m_error(3)) < m_tolerance(3) &&
           std::abs(m_error(4)) < m_tolerance(4);
  }

  /**
   * Sets the pose error which is considered tolerable for use with
   * AtReference().
   *
   * @param poseTolerance Pose error which is tolerable.
   * @param leftVelocityTolerance Left velocity error which is tolerable.
   * @param rightVelocityTolerance Right velocity error which is tolerable.
   */
  void SetTolerance(const Pose2d& poseTolerance,
                    mp::quantity<mp::m / mp::s> leftVelocityTolerance,
                    mp::quantity<mp::m / mp::s> rightVelocityTolerance) {
    m_tolerance = Eigen::Vector<double, 5>{
        mp::value(poseTolerance.X()), mp::value(poseTolerance.Y()),
        mp::value(poseTolerance.Rotation().Radians()),
        mp::value(leftVelocityTolerance), mp::value(rightVelocityTolerance)};
  }

  /**
   * Returns the left and right output voltages of the LTV controller.
   *
   * The reference pose, linear velocity, and angular velocity should come from
   * a drivetrain trajectory.
   *
   * @param currentPose      The current pose.
   * @param leftVelocity     The current left velocity.
   * @param rightVelocity    The current right velocity.
   * @param poseRef          The desired pose.
   * @param leftVelocityRef  The desired left velocity.
   * @param rightVelocityRef The desired right velocity.
   */
  DifferentialDriveWheelVoltages Calculate(
      const Pose2d& currentPose, mp::quantity<mp::m / mp::s> leftVelocity,
      mp::quantity<mp::m / mp::s> rightVelocity, const Pose2d& poseRef,
      mp::quantity<mp::m / mp::s> leftVelocityRef,
      mp::quantity<mp::m / mp::s> rightVelocityRef);

  /**
   * Returns the left and right output voltages of the LTV controller.
   *
   * The reference pose, linear velocity, and angular velocity should come from
   * a drivetrain trajectory.
   *
   * @param currentPose  The current pose.
   * @param leftVelocity The left velocity.
   * @param rightVelocity The right velocity.
   * @param desiredState The desired pose, linear velocity, and angular velocity
   *                     from a trajectory.
   */
  DifferentialDriveWheelVoltages Calculate(
      const Pose2d& currentPose, mp::quantity<mp::m / mp::s> leftVelocity,
      mp::quantity<mp::m / mp::s> rightVelocity,
      const Trajectory::State& desiredState) {
    // v = (v_r + v_l) / 2     (1)
    // w = (v_r - v_l) / (2r)  (2)
    // k = w / v               (3)
    //
    // v_l = v - wr
    // v_l = v - (vk)r
    // v_l = v(1 - kr)
    //
    // v_r = v + wr
    // v_r = v + (vk)r
    // v_r = v(1 + kr)
    return Calculate(
        currentPose, leftVelocity, rightVelocity, desiredState.pose,
        desiredState.velocity * (1 - (desiredState.curvature / (1.0 * mp::rad) *
                                      m_trackwidth / 2.0)),
        desiredState.velocity * (1 + (desiredState.curvature / (1.0 * mp::rad) *
                                      m_trackwidth / 2.0)));
  }

 private:
  mp::quantity<mp::m> m_trackwidth;

  // Continuous velocity dynamics
  Eigen::Matrix<double, 2, 2> m_A;
  Eigen::Matrix<double, 2, 2> m_B;

  // LQR cost matrices
  Eigen::Matrix<double, 5, 5> m_Q;
  Eigen::Matrix<double, 2, 2> m_R;

  mp::quantity<mp::s> m_dt;

  Eigen::Vector<double, 5> m_error;
  Eigen::Vector<double, 5> m_tolerance;
};

}  // namespace frc
