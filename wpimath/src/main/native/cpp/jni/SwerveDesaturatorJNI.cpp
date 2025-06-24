// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <jni.h>

#include <wpi/SmallVector.h>
#include <wpi/jni_util.h>

#include "edu_wpi_first_math_jni_SwerveDesaturatorJNI.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/SD2.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/time.h"
#include "units/velocity.h"

extern "C" {

/*
 * Class:     edu_wpi_first_math_jni_SwerveDesaturatorJNI
 * Method:    desaturatedDiscretize
 * Signature: (DDDDD[D[D)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_math_jni_SwerveDesaturatorJNI_desaturatedDiscretize
  (JNIEnv* env, jclass, jdouble vx, jdouble vy, jdouble omega, jdouble deltaT,
   jdouble v_max, jdoubleArray positions, jdoubleArray outputSpeeds)
{
  frc::ChassisSpeeds speeds{vx * 1_mps, vy * 1_mps, omega * 1_rad_per_s};
  units::second_t dt = deltaT * 1_s;
  units::meters_per_second_t maxModuleSpeed = v_max * 1_mps;
  wpi::java::JSpan<jdouble> positionsSpan{env, positions};
  wpi::SmallVector<frc::Translation2d, 4> modules;
  for (size_t i = 0; i < positionsSpan.size() / 2; ++i) {
    modules.emplace_back(positionsSpan[2 * i] * 1_m,
                         positionsSpan[2 * i + 1] * 1_m);
  }

  auto result = sd2::DesaturatedDiscretize(speeds, dt, maxModuleSpeed, modules);

  wpi::array buf{result.vx.value(), result.vy.value(), result.omega.value()};
  env->SetDoubleArrayRegion(outputSpeeds, 0, 3, buf.data());
}

}  // extern "C"
