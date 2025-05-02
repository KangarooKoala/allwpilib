// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/trajectory/Trajectory.h"

#include <wpi/json.h>

using namespace frc;

void frc::to_json(wpi::json& json, const Trajectory::State& state) {
  json = wpi::json{{"time", mp::value(state.t)},
                   {"velocity", mp::value(state.velocity)},
                   {"acceleration", mp::value(state.acceleration)},
                   {"pose", state.pose},
                   {"curvature", mp::value(state.curvature)}};
}

void frc::from_json(const wpi::json& json, Trajectory::State& state) {
  state.pose = json.at("pose").get<Pose2d>();
  state.t = json.at("time").get<double>() * mp::s;
  state.velocity = json.at("velocity").get<double>() * mp::m / mp::s;
  state.acceleration = json.at("acceleration").get<double>() * mp::m / mp::s2;
  state.curvature = json.at("curvature").get<double>() * mp::rad / mp::m;
}
