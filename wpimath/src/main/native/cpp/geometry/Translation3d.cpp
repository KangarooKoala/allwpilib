// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/Translation3d.h"

#include <wpi/json.h>

void frc::to_json(wpi::json& json, const Translation3d& translation) {
  json = wpi::json{{"x", mp::value(translation.X())},
                   {"y", mp::value(translation.Y())},
                   {"z", mp::value(translation.Z())}};
}

void frc::from_json(const wpi::json& json, Translation3d& translation) {
  translation = Translation3d{json.at("x").get<double>() * mp::m,
                              json.at("y").get<double>() * mp::m,
                              json.at("z").get<double>() * mp::m};
}
