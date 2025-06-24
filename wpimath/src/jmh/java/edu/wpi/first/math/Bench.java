// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDesaturator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import org.openjdk.jmh.annotations.Benchmark;
import org.openjdk.jmh.annotations.Scope;
import org.openjdk.jmh.annotations.State;
import org.openjdk.jmh.infra.Blackhole;

@State(Scope.Benchmark)
public class Bench {
    private static final ChassisSpeeds speeds = new ChassisSpeeds(5, -14, 30);
    private static final double dt = 0.02;
    private static final double maxModuleSpeed = 1.3;
    private static final Translation2d[] modules = {
        new Translation2d(0.5, 0), new Translation2d(0.5, 0),
        new Translation2d(0.5, 0.2), new Translation2d(0.5, 0.2)
    };

    @Benchmark
    public void pureJava(Blackhole bh) {
        bh.consume(SwerveDesaturator.desaturatedDiscretize(speeds, dt, maxModuleSpeed, modules));
    }

    @Benchmark
    public void jni(Blackhole bh) {
        bh.consume(SwerveDesaturator.desaturatedDiscretizeJNI(speeds, dt, maxModuleSpeed, modules));
    }
}
