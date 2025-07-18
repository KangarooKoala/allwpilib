// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include <vector>

#include <Eigen/QR>
#include <gtest/gtest.h>

#include "frc/EigenCore.h"
#include "frc/StateSpaceUtil.h"
#include "frc/estimator/ExtendedKalmanFilter.h"
#include "frc/system/NumericalJacobian.h"
#include "frc/system/plant/DCMotor.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/units.h"

namespace {

frc::Vectord<5> Dynamics(const frc::Vectord<5>& x, const frc::Vectord<2>& u) {
  auto motors = frc::DCMotor::CIM(2);

  // constexpr double Glow = 15.32;       // Low gear ratio
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

frc::Vectord<3> LocalMeasurementModel(
    const frc::Vectord<5>& x, [[maybe_unused]] const frc::Vectord<2>& u) {
  return frc::Vectord<3>{x(2), x(3), x(4)};
}

frc::Vectord<5> GlobalMeasurementModel(
    const frc::Vectord<5>& x, [[maybe_unused]] const frc::Vectord<2>& u) {
  return frc::Vectord<5>{x(0), x(1), x(2), x(3), x(4)};
}
}  // namespace

TEST(ExtendedKalmanFilterTest, Init) {
  constexpr auto dt = 0.00505 * mp::s;

  frc::ExtendedKalmanFilter<5, 2, 3> observer{Dynamics,
                                              LocalMeasurementModel,
                                              {0.5, 0.5, 10.0, 1.0, 1.0},
                                              {0.0001, 0.01, 0.01},
                                              dt};
  frc::Vectord<2> u{12.0, 12.0};
  observer.Predict(u, dt);

  auto localY = LocalMeasurementModel(observer.Xhat(), u);
  observer.Correct(u, localY);

  auto globalY = GlobalMeasurementModel(observer.Xhat(), u);
  auto R = frc::MakeCovMatrix(0.01, 0.01, 0.0001, 0.01, 0.01);
  observer.Correct<5>(u, globalY, GlobalMeasurementModel, R);
}

TEST(ExtendedKalmanFilterTest, Convergence) {
  constexpr auto dt = 0.00505 * mp::s;
  constexpr auto rb = 0.8382 * mp::m / 2.0;  // Robot radius

  frc::ExtendedKalmanFilter<5, 2, 3> observer{Dynamics,
                                              LocalMeasurementModel,
                                              {0.5, 0.5, 10.0, 1.0, 1.0},
                                              {0.0001, 0.5, 0.5},
                                              dt};

  auto waypoints = std::vector<frc::Pose2d>{
      frc::Pose2d{2.75 * mp::m, 22.521 * mp::m, 0.0 * mp::rad},
      frc::Pose2d{24.73 * mp::m, 19.68 * mp::m, 5.846 * mp::rad}};
  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      waypoints, {8.8 * mp::m / mp::s, 0.1 * mp::m / mp::s2});

  frc::Vectord<5> r = frc::Vectord<5>::Zero();
  frc::Vectord<2> u = frc::Vectord<2>::Zero();

  auto B = frc::NumericalJacobianU<5, 5, 2>(Dynamics, frc::Vectord<5>::Zero(),
                                            frc::Vectord<2>::Zero());

  observer.SetXhat(frc::Vectord<5>{
      mp::value(trajectory.InitialPose().Translation().X()),
      mp::value(trajectory.InitialPose().Translation().Y()),
      mp::value(trajectory.InitialPose().Rotation().Radians()), 0.0, 0.0});

  auto totalTime = trajectory.TotalTime();
  for (size_t i = 0; i < mp::value(totalTime / dt); ++i) {
    auto ref = trajectory.Sample(dt * i);
    mp::quantity<mp::m / mp::s> vl =
        ref.velocity * (1 - mp::value(ref.curvature * rb));
    mp::quantity<mp::m / mp::s> vr =
        ref.velocity * (1 + mp::value(ref.curvature * rb));

    frc::Vectord<5> nextR{mp::value(ref.pose.Translation().X()),
                          mp::value(ref.pose.Translation().Y()),
                          mp::value(ref.pose.Rotation().Radians()),
                          mp::value(vl), mp::value(vr)};

    auto localY = LocalMeasurementModel(nextR, frc::Vectord<2>::Zero());
    observer.Correct(u, localY + frc::MakeWhiteNoiseVector(0.0001, 0.5, 0.5));

    frc::Vectord<5> rdot = (nextR - r) / mp::value(dt);
    u = B.householderQr().solve(rdot - Dynamics(r, frc::Vectord<2>::Zero()));

    observer.Predict(u, dt);

    r = nextR;
  }

  auto localY = LocalMeasurementModel(observer.Xhat(), u);
  observer.Correct(u, localY);

  auto globalY = GlobalMeasurementModel(observer.Xhat(), u);
  auto R = frc::MakeCovMatrix(0.01, 0.01, 0.0001, 0.5, 0.5);
  observer.Correct<5>(u, globalY, GlobalMeasurementModel, R);

  auto finalPosition = trajectory.Sample(trajectory.TotalTime());
  ASSERT_NEAR(mp::value(finalPosition.pose.Translation().X()), observer.Xhat(0),
              1.0);
  ASSERT_NEAR(mp::value(finalPosition.pose.Translation().Y()), observer.Xhat(1),
              1.0);
  ASSERT_NEAR(mp::value(finalPosition.pose.Rotation().Radians()),
              observer.Xhat(2), 1.0);
  ASSERT_NEAR(0.0, observer.Xhat(3), 1.0);
  ASSERT_NEAR(0.0, observer.Xhat(4), 1.0);
}
