// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>

#include "frc/units.h"

namespace frc {

/**
 * Holds the constants for a DC motor.
 */
class WPILIB_DLLEXPORT DCMotor {
 public:
  using radians_per_second_per_volt_t = mp::quantity<mp::rad / mp::s / mp::V>;
  using newton_meters_per_ampere_t = mp::quantity<mp::N * mp::m / mp::A>;

  /// Voltage at which the motor constants were measured.
  mp::quantity<mp::V> nominalVoltage;

  /// Torque when stalled.
  mp::quantity<mp::N * mp::m> stallTorque;

  /// Current draw when stalled.
  mp::quantity<mp::A> stallCurrent;

  /// Current draw under no load.
  mp::quantity<mp::A> freeCurrent;

  /// Angular velocity under no load.
  mp::quantity<mp::rad / mp::s> freeSpeed;

  /// Motor internal resistance.
  mp::quantity<mp::ohm> R;

  /// Motor velocity constant.
  radians_per_second_per_volt_t Kv;

  /// Motor torque constant.
  newton_meters_per_ampere_t Kt;

  /**
   * Constructs a DC motor.
   *
   * @param nominalVoltage Voltage at which the motor constants were measured.
   * @param stallTorque    Torque when stalled.
   * @param stallCurrent   Current draw when stalled.
   * @param freeCurrent    Current draw under no load.
   * @param freeSpeed      Angular velocity under no load.
   * @param numMotors      Number of motors in a gearbox.
   */
  constexpr DCMotor(mp::quantity<mp::V> nominalVoltage,
                    mp::quantity<mp::N * mp::m> stallTorque,
                    mp::quantity<mp::A> stallCurrent,
                    mp::quantity<mp::A> freeCurrent,
                    mp::quantity<mp::rad / mp::s> freeSpeed, int numMotors = 1)
      : nominalVoltage(nominalVoltage),
        stallTorque(stallTorque * numMotors),
        stallCurrent(stallCurrent * numMotors),
        freeCurrent(freeCurrent * numMotors),
        freeSpeed(freeSpeed),
        R(nominalVoltage / this->stallCurrent),
        Kv(freeSpeed / (nominalVoltage - R * this->freeCurrent)),
        Kt(this->stallTorque / this->stallCurrent) {}

  /**
   * Returns current drawn by motor with given speed and input voltage.
   *
   * @param speed        The current angular velocity of the motor.
   * @param inputVoltage The voltage being applied to the motor.
   */
  constexpr mp::quantity<mp::A> Current(
      mp::quantity<mp::rad / mp::s> speed,
      mp::quantity<mp::V> inputVoltage) const {
    return -1.0 / Kv / R * speed + 1.0 / R * inputVoltage;
  }

  /**
   * Returns current drawn by motor for a given torque.
   *
   * @param torque The torque produced by the motor.
   */
  constexpr mp::quantity<mp::A> Current(
      mp::quantity<mp::N * mp::m> torque) const {
    return torque / Kt;
  }

  /**
   * Returns torque produced by the motor with a given current.
   *
   * @param current     The current drawn by the motor.
   */
  constexpr mp::quantity<mp::N * mp::m> Torque(
      mp::quantity<mp::A> current) const {
    return current * Kt;
  }

  /**
   * Returns the voltage provided to the motor for a given torque and
   * angular velocity.
   *
   * @param torque      The torque produced by the motor.
   * @param speed       The current angular velocity of the motor.
   */
  constexpr mp::quantity<mp::V> Voltage(
      mp::quantity<mp::N * mp::m> torque,
      mp::quantity<mp::rad / mp::s> speed) const {
    return 1.0 / Kv * speed + 1.0 / Kt * R * torque;
  }

  /**
   * Returns the angular speed produced by the motor at a given torque and input
   * voltage.
   *
   * @param torque        The torque produced by the motor.
   * @param inputVoltage  The input voltage provided to the motor.
   */
  constexpr mp::quantity<mp::rad / mp::s> Speed(
      mp::quantity<mp::N * mp::m> torque,
      mp::quantity<mp::V> inputVoltage) const {
    return inputVoltage * Kv - 1.0 / Kt * torque * R * Kv;
  }

  /**
   * Returns a copy of this motor with the given gearbox reduction applied.
   *
   * @param gearboxReduction  The gearbox reduction.
   */
  constexpr DCMotor WithReduction(double gearboxReduction) {
    return DCMotor(nominalVoltage, stallTorque * gearboxReduction, stallCurrent,
                   freeCurrent, freeSpeed / gearboxReduction);
  }

  /**
   * Returns a gearbox of CIM motors.
   */
  static constexpr DCMotor CIM(int numMotors = 1) {
    return DCMotor(12.0 * mp::V, 2.42 * mp::N * mp::m, 133.0 * mp::A,
                   2.7 * mp::A, 5310.0 * mp::rev / mp::min, numMotors);
  }

  /**
   * Returns a gearbox of MiniCIM motors.
   */
  static constexpr DCMotor MiniCIM(int numMotors = 1) {
    return DCMotor(12.0 * mp::V, 1.41 * mp::N * mp::m, 89.0 * mp::A,
                   3.0 * mp::A, 5840.0 * mp::rev / mp::min, numMotors);
  }

  /**
   * Returns a gearbox of Bag motor motors.
   */
  static constexpr DCMotor Bag(int numMotors = 1) {
    return DCMotor(12.0 * mp::V, 0.43 * mp::N * mp::m, 53.0 * mp::A,
                   1.8 * mp::A, 13180.0 * mp::rev / mp::min, numMotors);
  }

  /**
   * Returns a gearbox of Vex 775 Pro motors.
   */
  static constexpr DCMotor Vex775Pro(int numMotors = 1) {
    return DCMotor(12.0 * mp::V, 0.71 * mp::N * mp::m, 134.0 * mp::A,
                   0.7 * mp::A, 18730.0 * mp::rev / mp::min, numMotors);
  }

  /**
   * Returns a gearbox of Andymark RS 775-125 motors.
   */
  static constexpr DCMotor RS775_125(int numMotors = 1) {
    return DCMotor(12.0 * mp::V, 0.28 * mp::N * mp::m, 18.0 * mp::A,
                   1.6 * mp::A, 5800.0 * mp::rev / mp::min, numMotors);
  }

  /**
   * Returns a gearbox of Banebots RS 775 motors.
   */
  static constexpr DCMotor BanebotsRS775(int numMotors = 1) {
    return DCMotor(12.0 * mp::V, 0.72 * mp::N * mp::m, 97.0 * mp::A,
                   2.7 * mp::A, 13050.0 * mp::rev / mp::min, numMotors);
  }

  /**
   * Returns a gearbox of Andymark 9015 motors.
   */
  static constexpr DCMotor Andymark9015(int numMotors = 1) {
    return DCMotor(12.0 * mp::V, 0.36 * mp::N * mp::m, 71.0 * mp::A,
                   3.7 * mp::A, 14270.0 * mp::rev / mp::min, numMotors);
  }

  /**
   * Returns a gearbox of Banebots RS 550 motors.
   */
  static constexpr DCMotor BanebotsRS550(int numMotors = 1) {
    return DCMotor(12.0 * mp::V, 0.38 * mp::N * mp::m, 84.0 * mp::A,
                   0.4 * mp::A, 19000.0 * mp::rev / mp::min, numMotors);
  }

  /**
   * Returns a gearbox of NEO brushless motors.
   */
  static constexpr DCMotor NEO(int numMotors = 1) {
    return DCMotor(12.0 * mp::V, 2.6 * mp::N * mp::m, 105.0 * mp::A,
                   1.8 * mp::A, 5676.0 * mp::rev / mp::min, numMotors);
  }

  /**
   * Returns a gearbox of NEO 550 brushless motors.
   */
  static constexpr DCMotor NEO550(int numMotors = 1) {
    return DCMotor(12.0 * mp::V, 0.97 * mp::N * mp::m, 100.0 * mp::A,
                   1.4 * mp::A, 11000.0 * mp::rev / mp::min, numMotors);
  }

  /**
   * Returns a gearbox of Falcon 500 brushless motors.
   */
  static constexpr DCMotor Falcon500(int numMotors = 1) {
    return DCMotor(12.0 * mp::V, 4.69 * mp::N * mp::m, 257.0 * mp::A,
                   1.5 * mp::A, 6380.0 * mp::rev / mp::min, numMotors);
  }

  /**
   * Return a gearbox of Falcon 500 motors with FOC (Field-Oriented Control)
   * enabled.
   */
  static constexpr DCMotor Falcon500FOC(int numMotors = 1) {
    // https://store.ctr-electronics.com/falcon-500-powered-by-talon-fx/
    return DCMotor(12.0 * mp::V, 5.84 * mp::N * mp::m, 304.0 * mp::A,
                   1.5 * mp::A, 6080.0 * mp::rev / mp::min, numMotors);
  }

  /**
   * Return a gearbox of Romi/TI_RSLK MAX motors.
   */
  static constexpr DCMotor RomiBuiltIn(int numMotors = 1) {
    // From https://www.pololu.com/product/1520/specs
    return DCMotor(4.5 * mp::V, 0.1765 * mp::N * mp::m, 1.25 * mp::A,
                   0.13 * mp::A, 150.0 * mp::rev / mp::min, numMotors);
  }

  /**
   * Return a gearbox of Kraken X60 brushless motors.
   */
  static constexpr DCMotor KrakenX60(int numMotors = 1) {
    // From https://store.ctr-electronics.com/announcing-kraken-x60/
    return DCMotor(12.0 * mp::V, 7.09 * mp::N * mp::m, 366.0 * mp::A,
                   2.0 * mp::A, 6000.0 * mp::rev / mp::min, numMotors);
  }

  /**
   * Return a gearbox of Kraken X60 brushless motors with FOC (Field-Oriented
   * Control) enabled.
   */
  static constexpr DCMotor KrakenX60FOC(int numMotors = 1) {
    // From https://store.ctr-electronics.com/announcing-kraken-x60/
    return DCMotor(12.0 * mp::V, 9.37 * mp::N * mp::m, 483.0 * mp::A,
                   2.0 * mp::A, 5800.0 * mp::rev / mp::min, numMotors);
  }

  /**
   * Return a gearbox of Neo Vortex brushless motors.
   */
  static constexpr DCMotor NeoVortex(int numMotors = 1) {
    // From https://www.revrobotics.com/next-generation-spark-neo/
    return DCMotor(12.0 * mp::V, 3.60 * mp::N * mp::m, 211.0 * mp::A,
                   3.615 * mp::A, 6784.0 * mp::rev / mp::min, numMotors);
  }
};

}  // namespace frc

#include "frc/system/plant/proto/DCMotorProto.h"
#include "frc/system/plant/struct/DCMotorStruct.h"
