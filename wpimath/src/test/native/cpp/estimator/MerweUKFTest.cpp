// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

#include <Eigen/QR>
#include <gtest/gtest.h>

#include "frc/EigenCore.h"
#include "frc/StateSpaceUtil.h"
#include "frc/estimator/AngleStatistics.h"
#include "frc/estimator/MerweUKF.h"
#include "frc/system/Discretization.h"
#include "frc/system/NumericalIntegration.h"
#include "frc/system/NumericalJacobian.h"
#include "frc/system/plant/DCMotor.h"
#include "frc/system/plant/LinearSystemId.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/units.h"

namespace {

// First test system, differential drive
frc::Vectord<5> DriveDynamics(const frc::Vectord<5>& x,
                              const frc::Vectord<2>& u) {
  auto motors = frc::DCMotor::CIM(2);

  // constexpr double Glow = 15.32;    // Low gear ratio
  constexpr double Ghigh = 7.08;             // High gear ratio
  constexpr auto rb = 0.8382 * mp::m / 2.0;  // Robot radius
  constexpr auto r = 0.0746125 * mp::m;      // Wheel radius
  constexpr auto m = 63.503 * mp::kg;        // Robot mass
  constexpr auto J = 5.6 * mp::kg * mp::m2;  // Robot moment of inertia

  auto C1 =
      -std::pow(Ghigh, 2) * motors.Kt / (motors.Kv * motors.R * mp::pow<2>(r));
  auto C2 = Ghigh * motors.Kt / (motors.R * r);
  auto k1 = (1 / m + mp::pow<2>(rb) / J);
  auto k2 = (1 / m - mp::pow<2>(rb) / J);

  mp::quantity<mp::m / mp::s> vl = x(3) * mp::m / mp::s;
  mp::quantity<mp::m / mp::s> vr = x(4) * mp::m / mp::s;
  mp::quantity<mp::V> Vl = u(0) * mp::V;
  mp::quantity<mp::V> Vr = u(1) * mp::V;

  auto v = 0.5 * (vl + vr);
  return frc::Vectord<5>{
      mp::value(v) * std::cos(x(2)), mp::value(v) * std::sin(x(2)),
      mp::value((vr - vl) / (2.0 * rb)),
      mp::value(k1) * (mp::value(C1 * vl) + mp::value(C2 * Vl)) +
          mp::value(k2) * (mp::value(C1 * vr) + mp::value(C2 * Vr)),
      mp::value(k2) * (mp::value(C1 * vl) + mp::value(C2 * Vl)) +
          mp::value(k1) * (mp::value(C1 * vr) + mp::value(C2 * Vr))};
}

frc::Vectord<3> DriveLocalMeasurementModel(
    const frc::Vectord<5>& x, [[maybe_unused]] const frc::Vectord<2>& u) {
  return frc::Vectord<3>{x(2), x(3), x(4)};
}

frc::Vectord<5> DriveGlobalMeasurementModel(
    const frc::Vectord<5>& x, [[maybe_unused]] const frc::Vectord<2>& u) {
  return frc::Vectord<5>{x(0), x(1), x(2), x(3), x(4)};
}

TEST(MerweUKFTest, DriveInit) {
  constexpr auto dt = 5.0 * mp::ms;

  frc::MerweUKF<5, 2, 3> observer{DriveDynamics,
                                  DriveLocalMeasurementModel,
                                  {0.5, 0.5, 10.0, 1.0, 1.0},
                                  {0.0001, 0.01, 0.01},
                                  frc::AngleMean<5, 2 * 5 + 1>(2),
                                  frc::AngleMean<3, 2 * 5 + 1>(0),
                                  frc::AngleResidual<5>(2),
                                  frc::AngleResidual<3>(0),
                                  frc::AngleAdd<5>(2),
                                  dt};
  frc::Vectord<2> u{12.0, 12.0};
  observer.Predict(u, dt);

  auto localY = DriveLocalMeasurementModel(observer.Xhat(), u);
  observer.Correct(u, localY);

  auto globalY = DriveGlobalMeasurementModel(observer.Xhat(), u);
  auto R = frc::MakeCovMatrix(0.01, 0.01, 0.0001, 0.01, 0.01);
  observer.Correct<5>(u, globalY, DriveGlobalMeasurementModel, R,
                      frc::AngleMean<5, 2 * 5 + 1>(2), frc::AngleResidual<5>(2),
                      frc::AngleResidual<5>(2), frc::AngleAdd<5>(2));
}

TEST(MerweUKFTest, DriveConvergence) {
  constexpr mp::quantity<mp::s> dt = 5.0 * mp::ms;
  constexpr auto rb = 0.8382 * mp::m / 2.0;  // Robot radius

  frc::MerweUKF<5, 2, 3> observer{DriveDynamics,
                                  DriveLocalMeasurementModel,
                                  {0.5, 0.5, 10.0, 1.0, 1.0},
                                  {0.0001, 0.5, 0.5},
                                  frc::AngleMean<5, 2 * 5 + 1>(2),
                                  frc::AngleMean<3, 2 * 5 + 1>(0),
                                  frc::AngleResidual<5>(2),
                                  frc::AngleResidual<3>(0),
                                  frc::AngleAdd<5>(2),
                                  dt};

  auto waypoints = std::vector<frc::Pose2d>{
      frc::Pose2d{2.75 * mp::m, 22.521 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{24.73 * mp::m, 19.68 * mp::m, 5.846 * mp::rad}};
  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      waypoints, {8.8 * mp::m / mp::s, 0.1 * mp::m / mp::s2});

  frc::Vectord<5> r = frc::Vectord<5>::Zero();
  frc::Vectord<2> u = frc::Vectord<2>::Zero();

  auto B = frc::NumericalJacobianU<5, 5, 2>(
      DriveDynamics, frc::Vectord<5>::Zero(), frc::Vectord<2>::Zero());

  observer.SetXhat(frc::Vectord<5>{
      mp::value(trajectory.InitialPose().Translation().X()),
      mp::value(trajectory.InitialPose().Translation().Y()),
      mp::value(trajectory.InitialPose().Rotation().Radians()), 0.0, 0.0});

  auto trueXhat = observer.Xhat();

  auto totalTime = trajectory.TotalTime();
  for (size_t i = 0; i < double{totalTime / dt}; ++i) {
    auto ref = trajectory.Sample(dt * i);
    mp::quantity<mp::m / mp::s> vl =
        ref.velocity * (1 - mp::value(ref.curvature * rb));
    mp::quantity<mp::m / mp::s> vr =
        ref.velocity * (1 + mp::value(ref.curvature * rb));

    frc::Vectord<5> nextR{mp::value(ref.pose.Translation().X()),
                          mp::value(ref.pose.Translation().Y()),
                          mp::value(ref.pose.Rotation().Radians()),
                          mp::value(vl), mp::value(vr)};

    auto localY = DriveLocalMeasurementModel(trueXhat, frc::Vectord<2>::Zero());
    observer.Correct(u, localY + frc::MakeWhiteNoiseVector(0.0001, 0.5, 0.5));

    frc::Vectord<5> rdot = (nextR - r) / mp::value(dt);
    u = B.householderQr().solve(rdot -
                                DriveDynamics(r, frc::Vectord<2>::Zero()));

    observer.Predict(u, dt);

    r = nextR;
    trueXhat = frc::RK4(DriveDynamics, trueXhat, u, dt);
  }

  auto localY = DriveLocalMeasurementModel(trueXhat, u);
  observer.Correct(u, localY);

  auto globalY = DriveGlobalMeasurementModel(trueXhat, u);
  auto R = frc::MakeCovMatrix(0.01, 0.01, 0.0001, 0.5, 0.5);
  observer.Correct<5>(u, globalY, DriveGlobalMeasurementModel, R,
                      frc::AngleMean<5, 2 * 5 + 1>(2), frc::AngleResidual<5>(2),
                      frc::AngleResidual<5>(2), frc::AngleAdd<5>(2)

  );

  // Increase in rotation error caused by change to spline parameterizer
  // constants
  auto finalPosition = trajectory.Sample(trajectory.TotalTime());
  EXPECT_NEAR(mp::value(finalPosition.pose.Translation().X()), observer.Xhat(0),
              0.055);
  EXPECT_NEAR(mp::value(finalPosition.pose.Translation().Y()), observer.Xhat(1),
              0.15);
  EXPECT_NEAR(mp::value(finalPosition.pose.Rotation().Radians()),
              observer.Xhat(2), 0.000015);
  EXPECT_NEAR(0.0, observer.Xhat(3), 0.1);
  EXPECT_NEAR(0.0, observer.Xhat(4), 0.1);
}

TEST(MerweUKFTest, LinearUKF) {
  constexpr mp::quantity<mp::s> dt = 20.0 * mp::ms;
  auto plant = frc::LinearSystemId::IdentifyVelocitySystem<mp::m>(
      0.02 * mp::V / (mp::m / mp::s), 0.006 * mp::V / (mp::m / mp::s2));
  frc::MerweUKF<1, 1, 1> observer{
      [&](const frc::Vectord<1>& x, const frc::Vectord<1>& u) {
        return plant.A() * x + plant.B() * u;
      },
      [&](const frc::Vectord<1>& x, const frc::Vectord<1>& u) {
        return plant.CalculateY(x, u);
      },
      {0.05},
      {1.0},
      dt};

  frc::Matrixd<1, 1> discA;
  frc::Matrixd<1, 1> discB;
  frc::DiscretizeAB<1, 1>(plant.A(), plant.B(), dt, &discA, &discB);

  frc::Vectord<1> ref{100.0};
  frc::Vectord<1> u{0.0};

  for (int i = 0; i < 2.0 / mp::value(dt); ++i) {
    observer.Predict(u, dt);

    u = discB.householderQr().solve(ref - discA * ref);
  }

  EXPECT_NEAR(ref(0, 0), observer.Xhat(0), 5);
}

TEST(MerweUKFTest, RoundTripP) {
  constexpr auto dt = 5.0 * mp::ms;

  frc::MerweUKF<2, 2, 2> observer{
      [](const frc::Vectord<2>& x, const frc::Vectord<2>& u) { return x; },
      [](const frc::Vectord<2>& x, const frc::Vectord<2>& u) { return x; },
      {0.0, 0.0},
      {0.0, 0.0},
      dt};

  frc::Matrixd<2, 2> P({{2, 1}, {1, 2}});
  observer.SetP(P);

  ASSERT_TRUE(observer.P().isApprox(P));
}

// Second system, single motor feedforward estimator
frc::Vectord<4> MotorDynamics(const frc::Vectord<4>& x,
                              const frc::Vectord<1>& u) {
  double v = x(1);
  double kV = x(2);
  double kA = x(3);

  double V = u(0);

  double a = -kV / kA * v + 1.0 / kA * V;
  return frc::Vectord<4>{v, a, 0.0, 0.0};
}

frc::Vectord<3> MotorMeasurementModel(const frc::Vectord<4>& x,
                                      const frc::Vectord<1>& u) {
  double p = x(0);
  double v = x(1);
  double kV = x(2);
  double kA = x(3);

  double V = u(0);

  double a = -kV / kA * v + 1.0 / kA * V;
  return frc::Vectord<3>{p, v, a};
}

frc::Vectord<1> MotorControlInput(double t) {
  return frc::Vectord<1>{
      std::clamp(8 * std::sin(std::numbers::pi * std::sqrt(2.0) * t) +
                     6 * std::sin(std::numbers::pi * std::sqrt(3.0) * t) +
                     4 * std::sin(std::numbers::pi * std::sqrt(5.0) * t),
                 -12.0, 12.0)};
}

TEST(MerweUKFTest, MotorConvergence) {
  constexpr mp::quantity<mp::s> dt = 10.0 * mp::ms;
  constexpr int steps = 500;
  constexpr double true_kV = 3;
  constexpr double true_kA = 0.2;

  constexpr double pos_stddev = 0.02;
  constexpr double vel_stddev = 0.1;
  constexpr double accel_stddev = 0.1;

  std::vector<frc::Vectord<4>> states(steps + 1);
  std::vector<frc::Vectord<1>> inputs(steps);
  std::vector<frc::Vectord<3>> measurements(steps);
  states[0] = frc::Vectord<4>{{0.0}, {0.0}, {true_kV}, {true_kA}};

  constexpr frc::Matrixd<4, 4> A{{0.0, 1.0, 0.0, 0.0},
                                 {0.0, -true_kV / true_kA, 0.0, 0.0},
                                 {0.0, 0.0, 0.0, 0.0},
                                 {0.0, 0.0, 0.0, 0.0}};
  constexpr frc::Matrixd<4, 1> B{{0.0}, {1.0 / true_kA}, {0.0}, {0.0}};

  frc::Matrixd<4, 4> discA;
  frc::Matrixd<4, 1> discB;
  frc::DiscretizeAB(A, B, dt, &discA, &discB);

  for (int i = 0; i < steps; ++i) {
    inputs[i] = MotorControlInput(i * mp::value(dt));
    states[i + 1] = discA * states[i] + discB * inputs[i];
    measurements[i] =
        MotorMeasurementModel(states[i + 1], inputs[i]) +
        frc::MakeWhiteNoiseVector(pos_stddev, vel_stddev, accel_stddev);
  }

  frc::Vectord<4> P0{0.001, 0.001, 10, 10};

  frc::MerweUKF<4, 1, 3> observer{
      MotorDynamics, MotorMeasurementModel, wpi::array{0.1, 1.0, 1e-10, 1e-10},
      wpi::array{pos_stddev, vel_stddev, accel_stddev}, dt};

  observer.SetXhat(frc::Vectord<4>{0.0, 0.0, 2.0, 2.0});
  observer.SetP(P0.asDiagonal());

  for (int i = 0; i < steps; ++i) {
    observer.Predict(inputs[i], dt);
    observer.Correct(inputs[i], measurements[i]);
  }

  EXPECT_NEAR(true_kV, observer.Xhat(2), true_kV * 0.5);
  EXPECT_NEAR(true_kA, observer.Xhat(3), true_kA * 0.5);
}

}  // namespace
