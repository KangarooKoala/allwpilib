// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDesaturator;
import java.util.concurrent.TimeUnit;
import org.openjdk.jmh.annotations.Benchmark;
import org.openjdk.jmh.annotations.BenchmarkMode;
import org.openjdk.jmh.annotations.Mode;
import org.openjdk.jmh.annotations.OutputTimeUnit;
import org.openjdk.jmh.profile.GCProfiler;
import org.openjdk.jmh.runner.Runner;
import org.openjdk.jmh.runner.RunnerException;
import org.openjdk.jmh.runner.options.Options;
import org.openjdk.jmh.runner.options.OptionsBuilder;
import org.openjdk.jmh.runner.options.TimeValue;

public class DevMain {
  private static final ChassisSpeeds speeds = new ChassisSpeeds(5, -14, 30);
  private static final double dt = 0.02;
  private static final double maxModuleSpeed = 1.3;
  private static final Translation2d[] modules = {
    new Translation2d(0.5, 0), new Translation2d(0.5, 0),
    new Translation2d(0.5, 0.2), new Translation2d(0.5, 0.2)
  };

  /**
   * Main function.
   *
   * @param args The (unused) arguments to the program.
   */
  public static void main(String... args) throws RunnerException {
    Options opt =
        new OptionsBuilder()
            .include(DevMain.class.getSimpleName())
            .addProfiler(GCProfiler.class)
            .forks(1)
            .warmupIterations(2)
            .warmupTime(TimeValue.seconds(3))
            .measurementIterations(3)
            .measurementTime(TimeValue.seconds(3))
            .build();

    new Runner(opt).run();
  }

  @Benchmark
  @BenchmarkMode(Mode.AverageTime)
  @OutputTimeUnit(TimeUnit.MICROSECONDS)
  public ChassisSpeeds pureJava() {
    return SwerveDesaturator.desaturatedDiscretize(speeds, dt, maxModuleSpeed, modules);
  }

  @Benchmark
  @BenchmarkMode(Mode.AverageTime)
  @OutputTimeUnit(TimeUnit.MICROSECONDS)
  public ChassisSpeeds jni() {
    return SwerveDesaturator.desaturatedDiscretizeJNI(speeds, dt, maxModuleSpeed, modules);
  }
}
