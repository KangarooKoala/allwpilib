// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.kinematics;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import org.junit.jupiter.api.Test;

class SwerveDesaturatorTest {
  private static final double kEpsilon = 1e-9;

  private static ChassisSpeeds undiscretize(ChassisSpeeds speeds, double dtSeconds) {
    var twist =
        new Twist2d(
            speeds.vxMetersPerSecond * dtSeconds,
            speeds.vyMetersPerSecond * dtSeconds,
            speeds.omegaRadiansPerSecond * dtSeconds);
    var pose = Pose2d.kZero.exp(twist);

    return new ChassisSpeeds(
        pose.getX() / dtSeconds,
        pose.getY() / dtSeconds,
        pose.getRotation().getRadians() / dtSeconds);
  }

  private static void assertScaled(ChassisSpeeds lhs, ChassisSpeeds rhs) {
    String errorMessage = "Chassis speeds are not scalar multiples! lhs: " + lhs + ", rhs: " + rhs;
    if (Math.abs(lhs.vxMetersPerSecond) < kEpsilon
        && Math.abs(lhs.vyMetersPerSecond) < kEpsilon
        && Math.abs(lhs.omegaRadiansPerSecond) < kEpsilon) {
      // lhs is zero
      return;
    }
    var scaled = lhs.times(rhs.vxMetersPerSecond / lhs.vxMetersPerSecond);
    assertAll(
        errorMessage,
        () -> assertEquals(scaled.vxMetersPerSecond, rhs.vxMetersPerSecond, kEpsilon),
        () -> assertEquals(scaled.vyMetersPerSecond, rhs.vyMetersPerSecond, kEpsilon),
        () -> assertEquals(scaled.omegaRadiansPerSecond, rhs.omegaRadiansPerSecond, kEpsilon));
  }

  private static void testUnsaturated(
      ChassisSpeeds speeds, double dt, double maxModuleSpeed, Translation2d... modules) {
    // var outputSpeeds = SwerveDesaturator.desaturatedDiscretize(speeds, dt, maxModuleSpeed,
    // modules);
    var outputSpeeds =
        SwerveDesaturator.desaturatedDiscretizeJNI(speeds, dt, maxModuleSpeed, modules);
    var undiscretized = undiscretize(outputSpeeds, dt);

    assertAll(
        "Output speeds " + outputSpeeds + " did not undiscretize to expected " + speeds,
        () -> assertEquals(speeds.vxMetersPerSecond, undiscretized.vxMetersPerSecond, kEpsilon),
        () -> assertEquals(speeds.vyMetersPerSecond, undiscretized.vyMetersPerSecond, kEpsilon),
        () ->
            assertEquals(
                speeds.omegaRadiansPerSecond, undiscretized.omegaRadiansPerSecond, kEpsilon));
  }

  private static void testSaturated(
      ChassisSpeeds speeds, double dt, double maxModuleSpeed, Translation2d... modules) {
    var kinematics = new SwerveDriveKinematics(modules);

    // var outputSpeeds = SwerveDesaturator.desaturatedDiscretize(speeds, dt, maxModuleSpeed,
    // modules);
    var outputSpeeds =
        SwerveDesaturator.desaturatedDiscretizeJNI(speeds, dt, maxModuleSpeed, modules);
    var outputStates = kinematics.toSwerveModuleStates(outputSpeeds);

    double realMaxSpeed = 0.0;
    for (var module : outputStates) {
      if (module.speedMetersPerSecond > realMaxSpeed) {
        realMaxSpeed = module.speedMetersPerSecond;
      }
    }
    final double capturableRealMaxSpeed = realMaxSpeed;

    assertAll(
        "Invalid outputSpeeds " + outputSpeeds,
        () -> assertEquals(maxModuleSpeed, capturableRealMaxSpeed, kEpsilon),
        () -> assertScaled(speeds, undiscretize(outputSpeeds, dt)));
  }

  @Test
  void testStraightUnsaturated() {
    var speeds = new ChassisSpeeds(0.5, 0.0, 0.0);
    double dt = 0.02;
    double maxModuleSpeed = 1.0;
    Translation2d[] modules = {
      new Translation2d(1, 1),
      new Translation2d(1, -1),
      new Translation2d(-1, 1),
      new Translation2d(-1, -1)
    };

    testUnsaturated(speeds, dt, maxModuleSpeed, modules);
  }

  @Test
  void testStraightAllSaturated() {
    var speeds = new ChassisSpeeds(2.0, 1.0, 0.0);
    double dt = 0.02;
    double maxModuleSpeed = 1.0;
    Translation2d[] modules = {
      new Translation2d(1, 1),
      new Translation2d(1, -1),
      new Translation2d(-1, 1),
      new Translation2d(-1, -1)
    };

    testSaturated(speeds, dt, maxModuleSpeed, modules);
  }

  @Test
  void testCurvedUnsaturated() {
    var speeds = new ChassisSpeeds(0.5, 0.0, 0.1);
    double dt = 0.02;
    double maxModuleSpeed = 1.0;
    Translation2d[] modules = {
      new Translation2d(1, 1), new Translation2d(1, -1),
      new Translation2d(-1, 1), new Translation2d(-1, -1)
    };

    testUnsaturated(speeds, dt, maxModuleSpeed, modules);
  }

  @Test
  void testCurvedOneSaturated() {
    var speeds = new ChassisSpeeds(0.5, -0.5, 0.5);
    double dt = 0.02;
    double maxModuleSpeed = 1.0;
    Translation2d[] modules = {
      new Translation2d(1, 1), new Translation2d(1, -1),
      new Translation2d(-1, 1), new Translation2d(-1, -1)
    };

    testSaturated(speeds, dt, maxModuleSpeed, modules);
  }

  @Test
  void testCurvedAllSaturated() {
    var speeds = new ChassisSpeeds(2.0, 1.0, 0.1);
    double dt = 0.02;
    double maxModuleSpeed = 1.0;
    Translation2d[] modules = {
      new Translation2d(1, 1), new Translation2d(1, -1),
      new Translation2d(-1, 1), new Translation2d(-1, -1)
    };

    testSaturated(speeds, dt, maxModuleSpeed, modules);
  }

  @Test
  void testReverseSlope() {
    var speeds = new ChassisSpeeds(6, -20, 20);
    double dt = 0.02;
    double maxModuleSpeed = 2.0;
    Translation2d[] modules = {
      new Translation2d(1, 0), new Translation2d(1, 0),
      new Translation2d(1, 0), new Translation2d(1, 0)
    };

    testSaturated(speeds, dt, maxModuleSpeed, modules);
  }

  @Test
  void testPositiveLocalMin() {
    var speeds = new ChassisSpeeds(6, -20, 20);
    double dt = 0.02;
    double maxModuleSpeed = 1.0;
    Translation2d[] modules = {
      new Translation2d(1, 0), new Translation2d(1, 0),
      new Translation2d(1, 0), new Translation2d(1, 0)
    };

    testSaturated(speeds, dt, maxModuleSpeed, modules);
  }

  @Test
  void testSeparateRanges() {
    var speeds = new ChassisSpeeds(9, -30, 30);
    double dt = 0.02;
    double maxModuleSpeed = 2.0;
    Translation2d[] modules = {
      new Translation2d(1, 0), new Translation2d(1, 0),
      new Translation2d(1, 0), new Translation2d(1, 0)
    };

    testUnsaturated(speeds, dt, maxModuleSpeed, modules);
  }

  @Test
  void testInvalidatedResult() {
    var speeds = new ChassisSpeeds(5, -14, 30);
    double dt = 0.02;
    double maxModuleSpeed = 1.3;
    Translation2d[] modules = {
      new Translation2d(0.5, 0), new Translation2d(0.5, 0),
      new Translation2d(0.5, 0.2), new Translation2d(0.5, 0.2)
    };

    testSaturated(speeds, dt, maxModuleSpeed, modules);
  }
}
