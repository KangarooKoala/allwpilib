// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.kinematics;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.jni.SwerveDesaturatorJNI;

/** Class for swerve desaturation. */
public final class SwerveDesaturator {
  /** Functional interface to store the error function. */
  @FunctionalInterface
  private interface ErrorFunction {
    /**
     * Calculates the error (how much squared module speed exceeds squared max speed).
     *
     * @param px The x position of the module, in meters.
     * @param py The y position of the module, in meters.
     * @param k The scaling factor of the chassis speeds (dimensionless).
     * @return The error.
     */
    double apply(double px, double py, double k);
  }

  /**
   * Checks if a double is (nearly) 0.
   *
   * @param x The number to check.
   * @return Whether the number is (nearly) 0.
   */
  private static boolean isZero(double x) {
    return Math.abs(x) < 1e-9;
  }

  /**
   * Discretizes a continuous-time chassis speed with saturation constraints.
   *
   * @param continuousSpeeds The continuous speeds.
   * @param dtSeconds The duration of the timestep the speeds should be applied for, in seconds.
   * @param maxModuleSpeedMetersPerSecond The max speed that a module can reach, in meters per
   *     second.
   * @param modules The positions of the swerve modules.
   * @return The discretized and desaturated chassis speeds.
   * @see ChassisSpeeds#discretize(ChassisSpeeds, double)
   * @see SwerveDriveKinematics#desaturateWheelSpeeds(SwerveModuleState[], double)
   */
  public static ChassisSpeeds desaturatedDiscretize(
      ChassisSpeeds continuousSpeeds,
      double dtSeconds,
      double maxModuleSpeedMetersPerSecond,
      Translation2d... modules) {
    double vx = continuousSpeeds.vxMetersPerSecond;
    double vy = continuousSpeeds.vyMetersPerSecond;
    double ω = continuousSpeeds.omegaRadiansPerSecond;
    double Δt = dtSeconds;
    double v_max = maxModuleSpeedMetersPerSecond;

    ErrorFunction error =
        (px, py, k) -> {
          double halfΔθ = ω * Δt / 2 * k;
          double halfΔθByTanHalfΔθ;
          if (!isZero(halfΔθ)) {
            halfΔθByTanHalfΔθ = halfΔθ / Math.tan(halfΔθ);
          } else {
            double halfΔθSq = halfΔθ * halfΔθ;
            halfΔθByTanHalfΔθ = 1.0 - 1.0 / 3.0 * halfΔθSq - 1.0 / 45.0 * halfΔθSq * halfΔθSq;
          }
          double v_chassis_x = halfΔθByTanHalfΔθ * k * vx + halfΔθ * k * vy;
          double v_chassis_y = -halfΔθ * k * vx + halfΔθByTanHalfΔθ * k * vy;
          double ω_chassis = k * ω;
          double v_module_x = v_chassis_x - ω_chassis * py;
          double v_module_y = v_chassis_y + ω_chassis * px;
          return (v_module_x * v_module_x + v_module_y * v_module_y) - v_max * v_max;
        };

    double maxK = 1;

    int firstVerifiedIndex = modules.length;
    for (int i = 0; i != firstVerifiedIndex; i = (i + 1) % modules.length) {
      Translation2d module = modules[i];
      double px = module.getX();
      double py = module.getY();

      double k = maxK;
      double y = error.apply(px, py, k);
      if (isZero(y) || y < 0) {
        if (firstVerifiedIndex >= modules.length) {
          firstVerifiedIndex = i;
        }
        continue;
      }

      double k_prev = k;
      double y_prev = y;
      k = k_prev - 1e-9;
      y = error.apply(px, py, k);

      // Secant method
      while (!isZero(y) && y > 0) {
        double secant = (y - y_prev) / (k - k_prev);
        double k_new;
        if (isZero(secant) || secant < 0) {
          // Only one root with positive slope between k and 0, so we can halve k
          k_new = 0.5 * k;
        } else {
          k_new = k - y / secant;
          if (k_new < 0.5 * k) {
            k_new = 0.5 * k;
          }
        }
        k_prev = k;
        y_prev = y;
        k = k_new;
        y = error.apply(px, py, k);
      }

      if (!isZero(y)) {
        double k_neg = k;
        double y_neg = y;
        double k_pos = k_prev;
        double y_pos = y_prev;

        while (true) {
          double k_new = (k_neg * y_pos - k_pos * y_neg) / (y_pos - y_neg);
          double y_new = error.apply(px, py, k_new);
          if (isZero(y_new)) {
            // y isn't used after this, so only update k
            k = k_new;
            break;
          }
          if (y_new < 0) {
            k_neg = k_new;
            y_neg = y_new;
          } else {
            k_pos = k_new;
            y_pos = y_new;
          }
        }
      }

      maxK = k;
      firstVerifiedIndex = i;
    }

    return ChassisSpeeds.discretize(maxK * vx, maxK * vy, maxK * ω, dtSeconds);
  }

  /**
   * Discretizes a continuous-time chassis speed with saturation constraints.
   *
   * @param continuousSpeeds The continuous speeds.
   * @param dtSeconds The duration of the timestep the speeds should be applied for, in seconds.
   * @param maxModuleSpeedMetersPerSecond The max speed that a module can reach, in meters per
   *     second.
   * @param modules The positions of the swerve modules.
   * @return The discretized and desaturated chassis speeds.
   * @see ChassisSpeeds#discretize(ChassisSpeeds, double)
   * @see SwerveDriveKinematics#desaturateWheelSpeeds(SwerveModuleState[], double)
   */
  public static ChassisSpeeds desaturatedDiscretizeJNI(
      ChassisSpeeds continuousSpeeds,
      double dtSeconds,
      double maxModuleSpeedMetersPerSecond,
      Translation2d... modules) {
    double vx = continuousSpeeds.vxMetersPerSecond;
    double vy = continuousSpeeds.vyMetersPerSecond;
    double ω = continuousSpeeds.omegaRadiansPerSecond;
    double Δt = dtSeconds;
    double v_max = maxModuleSpeedMetersPerSecond;
    double[] positions = new double[2 * modules.length];
    for (int i = 0; i < modules.length; ++i) {
      positions[2 * i] = modules[i].getX();
      positions[2 * i + 1] = modules[i].getY();
    }

    double[] outputSpeeds = new double[3];
    SwerveDesaturatorJNI.desaturatedDiscretize(vx, vy, ω, Δt, v_max, positions, outputSpeeds);

    return new ChassisSpeeds(outputSpeeds[0], outputSpeeds[1], outputSpeeds[2]);
  }

  /** Utility class. */
  private SwerveDesaturator() {}
}
