// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <concepts>
#include <stdexcept>

#include <gcem.hpp>
#include <wpi/SymbolExports.h>

#include "frc/system/LinearSystem.h"
#include "frc/system/plant/DCMotor.h"
#include "frc/units.h"

namespace frc {
/**
 * Linear system ID utility functions.
 */
class WPILIB_DLLEXPORT LinearSystemId {
 public:
  template <mp::Unit auto Distance>
  using Velocity_t = mp::quantity<Distance / mp::s>;

  template <mp::Unit auto Distance>
  using Acceleration_t = mp::quantity<Distance / mp::s2>;

  /**
   * Create a state-space model of the elevator system. The states of the system
   * are [position, velocity]ᵀ, inputs are [voltage], and outputs are [position,
   * velocity]ᵀ.
   *
   * @param motor The motor (or gearbox) attached to the carriage.
   * @param mass The mass of the elevator carriage, in kilograms.
   * @param radius The radius of the elevator's driving drum, in meters.
   * @param gearing Gear ratio from motor to carriage.
   * @throws std::domain_error if mass <= 0, radius <= 0, or gearing <= 0.
   */
  static constexpr LinearSystem<2, 1, 2> ElevatorSystem(
      DCMotor motor, mp::quantity<mp::kg> mass, mp::quantity<mp::m> radius,
      double gearing) {
    if (mass <= 0.0 * mp::kg) {
      throw std::domain_error("mass must be greater than zero.");
    }
    if (radius <= 0.0 * mp::m) {
      throw std::domain_error("radius must be greater than zero.");
    }
    if (gearing <= 0.0) {
      throw std::domain_error("gearing must be greater than zero.");
    }

    Matrixd<2, 2> A{
        {0.0, 1.0},
        {0.0, mp::value(-gcem::pow(gearing, 2) * motor.Kt /
                        (motor.R * mp::pow<2>(radius) * mass * motor.Kv))}};
    Matrixd<2, 1> B{
        {0.0}, {mp::value(gearing * motor.Kt / (motor.R * radius * mass))}};
    Matrixd<2, 2> C{{1.0, 0.0}, {0.0, 1.0}};
    Matrixd<2, 1> D{{0.0}, {0.0}};

    return LinearSystem<2, 1, 2>(A, B, C, D);
  }

  /**
   * Create a state-space model of a single-jointed arm system.The states of the
   * system are [angle, angular velocity]ᵀ, inputs are [voltage], and outputs
   * are [angle, angular velocity]ᵀ.
   *
   * @param motor The motor (or gearbox) attached to the arm.
   * @param J The moment of inertia J of the arm.
   * @param gearing Gear ratio from motor to arm.
   * @throws std::domain_error if J <= 0 or gearing <= 0.
   */
  static constexpr LinearSystem<2, 1, 2> SingleJointedArmSystem(
      DCMotor motor, mp::quantity<mp::kg * mp::m2> J, double gearing) {
    if (J <= 0.0 * mp::kg * mp::m2) {
      throw std::domain_error("J must be greater than zero.");
    }
    if (gearing <= 0.0) {
      throw std::domain_error("gearing must be greater than zero.");
    }

    Matrixd<2, 2> A{{0.0, 1.0},
                    {0.0, mp::value(-gcem::pow(gearing, 2) * motor.Kt /
                                    (motor.Kv * motor.R * J))}};
    Matrixd<2, 1> B{{0.0}, {mp::value(gearing * motor.Kt / (motor.R * J))}};
    Matrixd<2, 2> C{{1.0, 0.0}, {0.0, 1.0}};
    Matrixd<2, 1> D{{0.0}, {0.0}};

    return LinearSystem<2, 1, 2>(A, B, C, D);
  }

  /**
   * Create a state-space model for a 1 DOF velocity system from its kV
   * (volts/(unit/sec)) and kA (volts/(unit/sec²)). These constants can be
   * found using SysId. The states of the system are [velocity], inputs are
   * [voltage], and outputs are [velocity].
   *
   * You MUST use an SI unit (i.e. meters or radians) for the Distance template
   * argument. You may still use non-SI units (such as feet or inches) for the
   * actual method arguments; they will automatically be converted to SI
   * internally.
   *
   * The parameters provided by the user are from this feedforward model:
   *
   * u = K_v v + K_a a
   *
   * @param kV The velocity gain, in volts/(unit/sec).
   * @param kA The acceleration gain, in volts/(unit/sec²).
   * @throws std::domain_error if kV < 0 or kA <= 0.
   * @see <a
   * href="https://github.com/wpilibsuite/sysid">https://github.com/wpilibsuite/sysid</a>
   */
  template <mp::Unit auto Distance>
    requires(Distance == mp::m) || (Distance == mp::rad)
  static constexpr LinearSystem<1, 1, 1> IdentifyVelocitySystem(
      mp::quantity<mp::V / Velocity_t<Distance>::unit> kV,
      mp::quantity<mp::V / Acceleration_t<Distance>::unit> kA) {
    if (kV < 0.0 * decltype(kV)::unit) {
      throw std::domain_error("Kv must be greater than or equal to zero.");
    }
    if (kA <= 0.0 * decltype(kA)::unit) {
      throw std::domain_error("Ka must be greater than zero.");
    }

    Matrixd<1, 1> A{{-mp::value(kV) / mp::value(kA)}};
    Matrixd<1, 1> B{{1.0 / mp::value(kA)}};
    Matrixd<1, 1> C{{1.0}};
    Matrixd<1, 1> D{{0.0}};

    return LinearSystem<1, 1, 1>(A, B, C, D);
  }

  /**
   * Create a state-space model for a 1 DOF position system from its kV
   * (volts/(unit/sec)) and kA (volts/(unit/sec²)). These constants can be
   * found using SysId. the states of the system are [position, velocity]ᵀ,
   * inputs are [voltage], and outputs are [position, velocity]ᵀ.
   *
   * You MUST use an SI unit (i.e. meters or radians) for the Distance template
   * argument. You may still use non-SI units (such as feet or inches) for the
   * actual method arguments; they will automatically be converted to SI
   * internally.
   *
   * The parameters provided by the user are from this feedforward model:
   *
   * u = K_v v + K_a a
   *
   * @param kV The velocity gain, in volts/(unit/sec).
   * @param kA The acceleration gain, in volts/(unit/sec²).
   *
   * @throws std::domain_error if kV < 0 or kA <= 0.
   * @see <a
   * href="https://github.com/wpilibsuite/sysid">https://github.com/wpilibsuite/sysid</a>
   */
  template <mp::Unit auto Distance>
    requires(Distance == mp::m) || (Distance == mp::rad)
  static constexpr LinearSystem<2, 1, 2> IdentifyPositionSystem(
      mp::quantity<mp::V / Velocity_t<Distance>::unit> kV,
      mp::quantity<mp::V / Acceleration_t<Distance>::unit> kA) {
    if (kV < 0.0 * decltype(kV)::unit) {
      throw std::domain_error("Kv must be greater than or equal to zero.");
    }
    if (kA <= 0.0 * decltype(kA)::unit) {
      throw std::domain_error("Ka must be greater than zero.");
    }

    Matrixd<2, 2> A{{0.0, 1.0}, {0.0, -mp::value(kV) / mp::value(kA)}};
    Matrixd<2, 1> B{{0.0}, {1.0 / mp::value(kA)}};
    Matrixd<2, 2> C{{1.0, 0.0}, {0.0, 1.0}};
    Matrixd<2, 1> D{{0.0}, {0.0}};

    return LinearSystem<2, 1, 2>(A, B, C, D);
  }

  /**
   * Identify a differential drive drivetrain given the drivetrain's kV and kA
   * in both linear {(volts/(meter/sec), (volts/(meter/sec²))} and angular
   * {(volts/(radian/sec), (volts/(radian/sec²))} cases. These constants can be
   * found using SysId.
   *
   * States: [[left velocity], [right velocity]]<br>
   * Inputs: [[left voltage], [right voltage]]<br>
   * Outputs: [[left velocity], [right velocity]]
   *
   * @param kVLinear  The linear velocity gain in volts per (meters per second).
   * @param kALinear  The linear acceleration gain in volts per (meters per
   *                  second squared).
   * @param kVAngular The angular velocity gain in volts per (meters per
   *                  second).
   * @param kAAngular The angular acceleration gain in volts per (meters per
   *                  second squared).
   * @throws domain_error if kVLinear <= 0, kALinear <= 0, kVAngular <= 0,
   *         or kAAngular <= 0.
   * @see <a
   * href="https://github.com/wpilibsuite/sysid">https://github.com/wpilibsuite/sysid</a>
   */
  static constexpr LinearSystem<2, 2, 2> IdentifyDrivetrainSystem(
      mp::quantity<mp::V / (mp::m / mp::s)> kVLinear,
      mp::quantity<mp::V / (mp::m / mp::s2)> kALinear,
      mp::quantity<mp::V / (mp::m / mp::s)> kVAngular,
      mp::quantity<mp::V / (mp::m / mp::s2)> kAAngular) {
    if (kVLinear <= 0.0 * decltype(kVLinear)::unit) {
      throw std::domain_error("Kv,linear must be greater than zero.");
    }
    if (kALinear <= 0.0 * decltype(kALinear)::unit) {
      throw std::domain_error("Ka,linear must be greater than zero.");
    }
    if (kVAngular <= 0.0 * decltype(kVAngular)::unit) {
      throw std::domain_error("Kv,angular must be greater than zero.");
    }
    if (kAAngular <= 0.0 * decltype(kAAngular)::unit) {
      throw std::domain_error("Ka,angular must be greater than zero.");
    }

    double A1 = -(mp::value(kVLinear) / mp::value(kALinear) +
                  mp::value(kVAngular) / mp::value(kAAngular));
    double A2 = -(mp::value(kVLinear) / mp::value(kALinear) -
                  mp::value(kVAngular) / mp::value(kAAngular));
    double B1 = 1.0 / mp::value(kALinear) + 1.0 / mp::value(kAAngular);
    double B2 = 1.0 / mp::value(kALinear) - 1.0 / mp::value(kAAngular);

    A1 /= 2.0;
    A2 /= 2.0;
    B1 /= 2.0;
    B2 /= 2.0;

    Matrixd<2, 2> A{{A1, A2}, {A2, A1}};
    Matrixd<2, 2> B{{B1, B2}, {B2, B1}};
    Matrixd<2, 2> C{{1.0, 0.0}, {0.0, 1.0}};
    Matrixd<2, 2> D{{0.0, 0.0}, {0.0, 0.0}};

    return LinearSystem<2, 2, 2>(A, B, C, D);
  }

  /**
   * Identify a differential drive drivetrain given the drivetrain's kV and kA
   * in both linear {(volts/(meter/sec)), (volts/(meter/sec²))} and angular
   * {(volts/(radian/sec)), (volts/(radian/sec²))} cases. This can be found
   * using SysId.
   *
   * States: [[left velocity], [right velocity]]<br>
   * Inputs: [[left voltage], [right voltage]]<br>
   * Outputs: [[left velocity], [right velocity]]
   *
   * @param kVLinear   The linear velocity gain in volts per (meters per
   * second).
   * @param kALinear   The linear acceleration gain in volts per (meters per
   *                   second squared).
   * @param kVAngular  The angular velocity gain in volts per (radians per
   *                   second).
   * @param kAAngular  The angular acceleration gain in volts per (radians per
   *                   second squared).
   * @param trackwidth The distance between the differential drive's left and
   *                   right wheels, in meters.
   * @throws domain_error if kVLinear <= 0, kALinear <= 0, kVAngular <= 0,
   *         kAAngular <= 0, or trackwidth <= 0.
   * @see <a
   * href="https://github.com/wpilibsuite/sysid">https://github.com/wpilibsuite/sysid</a>
   */
  static constexpr LinearSystem<2, 2, 2> IdentifyDrivetrainSystem(
      mp::quantity<mp::V / (mp::m / mp::s)> kVLinear,
      mp::quantity<mp::V / (mp::m / mp::s2)> kALinear,
      mp::quantity<mp::V / (mp::rad / mp::s)> kVAngular,
      mp::quantity<mp::V / (mp::rad / mp::s2)> kAAngular,
      mp::quantity<mp::m> trackwidth) {
    if (kVLinear <= 0.0 * decltype(kVLinear)::unit) {
      throw std::domain_error("Kv,linear must be greater than zero.");
    }
    if (kALinear <= 0.0 * decltype(kALinear)::unit) {
      throw std::domain_error("Ka,linear must be greater than zero.");
    }
    if (kVAngular <= 0.0 * decltype(kVAngular)::unit) {
      throw std::domain_error("Kv,angular must be greater than zero.");
    }
    if (kAAngular <= 0.0 * decltype(kAAngular)::unit) {
      throw std::domain_error("Ka,angular must be greater than zero.");
    }
    if (trackwidth <= 0.0 * mp::m) {
      throw std::domain_error("r must be greater than zero.");
    }

    // We want to find a factor to include in Kv,angular that will convert
    // `u = Kv,angular omega` to `u = Kv,angular v`.
    //
    // v = omega r
    // omega = v/r
    // omega = 1/r v
    // omega = 1/(trackwidth/2) v
    // omega = 2/trackwidth v
    //
    // So multiplying by 2/trackwidth converts the angular gains from V/(rad/s)
    // to V/(m/s).
    return IdentifyDrivetrainSystem(
        kVLinear, kALinear, kVAngular * 2.0 / trackwidth * 1.0 * mp::rad,
        kAAngular * 2.0 / trackwidth * 1.0 * mp::rad);
  }

  /**
   * Create a state-space model of a flywheel system, the states of the system
   * are [angular velocity], inputs are [voltage], and outputs are [angular
   * velocity].
   *
   * @param motor The motor (or gearbox) attached to the flywheel.
   * @param J The moment of inertia J of the flywheel.
   * @param gearing Gear ratio from motor to flywheel.
   * @throws std::domain_error if J <= 0 or gearing <= 0.
   */
  static constexpr LinearSystem<1, 1, 1> FlywheelSystem(
      DCMotor motor, mp::quantity<mp::kg * mp::m2> J, double gearing) {
    if (J <= 0.0 * mp::kg * mp::m2) {
      throw std::domain_error("J must be greater than zero.");
    }
    if (gearing <= 0.0) {
      throw std::domain_error("gearing must be greater than zero.");
    }

    Matrixd<1, 1> A{{mp::value(-gcem::pow(gearing, 2) * motor.Kt /
                               (motor.Kv * motor.R * J))}};
    Matrixd<1, 1> B{{mp::value(gearing * motor.Kt / (motor.R * J))}};
    Matrixd<1, 1> C{{1.0}};
    Matrixd<1, 1> D{{0.0}};

    return LinearSystem<1, 1, 1>(A, B, C, D);
  }

  /**
   * Create a state-space model of a DC motor system. The states of the system
   * are [angular position, angular velocity]ᵀ, inputs are [voltage], and
   * outputs are [angular position, angular velocity]ᵀ.
   *
   * @param motor The motor (or gearbox) attached to the system.
   * @param J the moment of inertia J of the DC motor.
   * @param gearing Gear ratio from motor to output.
   * @throws std::domain_error if J <= 0 or gearing <= 0.
   * @see <a
   * href="https://github.com/wpilibsuite/sysid">https://github.com/wpilibsuite/sysid</a>
   */
  static constexpr LinearSystem<2, 1, 2> DCMotorSystem(
      DCMotor motor, mp::quantity<mp::kg * mp::m2> J, double gearing) {
    if (J <= 0.0 * mp::kg * mp::m2) {
      throw std::domain_error("J must be greater than zero.");
    }
    if (gearing <= 0.0) {
      throw std::domain_error("gearing must be greater than zero.");
    }

    Matrixd<2, 2> A{{0.0, 1.0},
                    {0.0, mp::value(-gcem::pow(gearing, 2) * motor.Kt /
                                    (motor.Kv * motor.R * J))}};
    Matrixd<2, 1> B{{0.0}, {mp::value(gearing * motor.Kt / (motor.R * J))}};
    Matrixd<2, 2> C{{1.0, 0.0}, {0.0, 1.0}};
    Matrixd<2, 1> D{{0.0}, {0.0}};

    return LinearSystem<2, 1, 2>(A, B, C, D);
  }

  /**
   * Create a state-space model of a DC motor system from its kV
   * (volts/(unit/sec)) and kA (volts/(unit/sec²)). These constants can be
   * found using SysId. the states of the system are [angular position, angular
   * velocity]ᵀ, inputs are [voltage], and outputs are [angular position,
   * angular velocity]ᵀ.
   *
   * You MUST use an SI unit (i.e. meters or radians) for the Distance template
   * argument. You may still use non-SI units (such as feet or inches) for the
   * actual method arguments; they will automatically be converted to SI
   * internally.
   *
   * The parameters provided by the user are from this feedforward model:
   *
   * u = K_v v + K_a a
   *
   * @param kV The velocity gain, in volts/(unit/sec).
   * @param kA The acceleration gain, in volts/(unit/sec²).
   *
   * @throws std::domain_error if kV < 0 or kA <= 0.
   */
  template <mp::Unit auto Distance>
    requires(Distance == mp::m) || (Distance == mp::rad)
  static constexpr LinearSystem<2, 1, 2> DCMotorSystem(
      mp::quantity<mp::V / Velocity_t<Distance>::unit> kV,
      mp::quantity<mp::V / Acceleration_t<Distance>::unit> kA) {
    if (kV < 0.0 * decltype(kV)::unit) {
      throw std::domain_error("Kv must be greater than or equal to zero.");
    }
    if (kA <= 0.0 * decltype(kA)::unit) {
      throw std::domain_error("Ka must be greater than zero.");
    }

    Matrixd<2, 2> A{{0.0, 1.0}, {0.0, -mp::value(kV) / mp::value(kA)}};
    Matrixd<2, 1> B{0.0, 1.0 / mp::value(kA)};
    Matrixd<2, 2> C{{1.0, 0.0}, {0.0, 1.0}};
    Matrixd<2, 1> D{{0.0}, {0.0}};

    return LinearSystem<2, 1, 2>(A, B, C, D);
  }

  /**
   * Create a state-space model of differential drive drivetrain. In this model,
   * the states are [left velocity, right velocity]ᵀ, the inputs are [left
   * voltage, right voltage], and the outputs are [left velocity, right
   * velocity]ᵀ.
   *
   * @param motor The motor (or gearbox) driving the drivetrain.
   * @param mass The mass of the robot in kilograms.
   * @param r The radius of the wheels in meters.
   * @param rb The radius of the base (half of the trackwidth), in meters.
   * @param J The moment of inertia of the robot.
   * @param gearing Gear ratio from motor to wheel.
   * @throws std::domain_error if mass <= 0, r <= 0, rb <= 0, J <= 0, or
   *         gearing <= 0.
   */
  static constexpr LinearSystem<2, 2, 2> DrivetrainVelocitySystem(
      const DCMotor& motor, mp::quantity<mp::kg> mass, mp::quantity<mp::m> r,
      mp::quantity<mp::m> rb, mp::quantity<mp::kg * mp::m2> J, double gearing) {
    if (mass <= 0.0 * mp::kg) {
      throw std::domain_error("mass must be greater than zero.");
    }
    if (r <= 0.0 * mp::m) {
      throw std::domain_error("r must be greater than zero.");
    }
    if (rb <= 0.0 * mp::m) {
      throw std::domain_error("rb must be greater than zero.");
    }
    if (J <= 0.0 * mp::kg * mp::m2) {
      throw std::domain_error("J must be greater than zero.");
    }
    if (gearing <= 0.0) {
      throw std::domain_error("gearing must be greater than zero.");
    }

    auto C1 = -gcem::pow(gearing, 2) * motor.Kt /
              (motor.Kv * motor.R * mp::pow<2>(r));
    auto C2 = gearing * motor.Kt / (motor.R * r);

    Matrixd<2, 2> A{{mp::value((1 / mass + mp::pow<2>(rb) / J) * C1),
                     mp::value((1 / mass - mp::pow<2>(rb) / J) * C1)},
                    {mp::value((1 / mass - mp::pow<2>(rb) / J) * C1),
                     mp::value((1 / mass + mp::pow<2>(rb) / J) * C1)}};
    Matrixd<2, 2> B{{mp::value((1 / mass + mp::pow<2>(rb) / J) * C2),
                     mp::value((1 / mass - mp::pow<2>(rb) / J) * C2)},
                    {mp::value((1 / mass - mp::pow<2>(rb) / J) * C2),
                     mp::value((1 / mass + mp::pow<2>(rb) / J) * C2)}};
    Matrixd<2, 2> C{{1.0, 0.0}, {0.0, 1.0}};
    Matrixd<2, 2> D{{0.0, 0.0}, {0.0, 0.0}};

    return LinearSystem<2, 2, 2>(A, B, C, D);
  }
};

}  // namespace frc
