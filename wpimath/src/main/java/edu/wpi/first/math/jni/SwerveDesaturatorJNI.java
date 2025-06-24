// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.jni;

/** SwerveDesaturator JNI. */
public final class SwerveDesaturatorJNI extends WPIMathJNI {
  /**
   * Discretizes a continuous-time chassis speed with saturation constraints.
   *
   * @param vx The continuous x velocity in meters per second.
   * @param vy The continuous y velocity in meters per second.
   * @param omega The rotational velocity in meters per second.
   * @param deltaT The duration of the timestep the speeds should be applied for.
   * @param v_max The max speed that a module can reach in meters per second.
   * @param positions Array containing the positions of the swerve modules, in meters. The x and y
   *     coordinates for a point appear together (i.e., [x1, y1, x2, y2, ...]).
   * @param outputSpeeds Array to store the speeds into. ([vx, vy, ω])
   */
  public static native void desaturatedDiscretize(
      double vx,
      double vy,
      double omega,
      double deltaT,
      double v_max,
      double[] positions,
      double[] outputSpeeds);

  /** Utility class. */
  private SwerveDesaturatorJNI() {}
}
