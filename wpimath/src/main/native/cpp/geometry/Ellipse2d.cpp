// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/Ellipse2d.h"

#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "frc/units.h"

using namespace frc;

Translation2d Ellipse2d::Nearest(const Translation2d& point) const {
  // Check if already in ellipse
  if (Contains(point)) {
    return point;
  }

  // Rotate the point by the inverse of the ellipse's rotation
  auto rotPoint =
      point.RotateAround(m_center.Translation(), -m_center.Rotation());

  // Find nearest point
  {
    namespace slp = sleipnir;

    sleipnir::OptimizationProblem problem;

    // Point on ellipse
    auto x = problem.DecisionVariable();
    x.SetValue(mp::value(rotPoint.X()));
    auto y = problem.DecisionVariable();
    y.SetValue(mp::value(rotPoint.Y()));

    problem.Minimize(slp::pow(x - mp::value(rotPoint.X()), 2) +
                     slp::pow(y - mp::value(rotPoint.Y()), 2));

    // (x − x_c)²/a² + (y − y_c)²/b² = 1
    // b²(x − x_c)² + a²(y − y_c)² = a²b²
    double a2 = mp::value(m_xSemiAxis) * mp::value(m_xSemiAxis);
    double b2 = mp::value(m_ySemiAxis) * mp::value(m_ySemiAxis);
    problem.SubjectTo(b2 * slp::pow(x - mp::value(m_center.X()), 2) +
                          a2 * slp::pow(y - mp::value(m_center.Y()), 2) ==
                      a2 * b2);

    problem.Solve();

    rotPoint = frc::Translation2d{x.Value() * mp::m, y.Value() * mp::m};
  }

  // Undo rotation
  return rotPoint.RotateAround(m_center.Translation(), m_center.Rotation());
}
