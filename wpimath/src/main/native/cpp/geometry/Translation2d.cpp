// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/Translation2d.h"

#include <wpi/json.h>

void frc::to_json(wpi::json& json, const Translation2d& translation) {
  json = wpi::json{{"x", mp::value(translation.X())},
                   {"y", mp::value(translation.Y())}};
}

void frc::from_json(const wpi::json& json, Translation2d& translation) {
  translation = Translation2d{json.at("x").get<double>() * mp::m,
                              json.at("y").get<double>() * mp::m};
}
