// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <jni.h>

#include <wpi/jni_util.h>

#include "edu_wpi_first_math_jni_Transform3dJNI.h"
#include "frc/geometry/Quaternion.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Transform3d.h"
#include "frc/geometry/Twist3d.h"
#include "frc/units.h"

using namespace wpi::java;

extern "C" {

/*
 * Class:     edu_wpi_first_math_jni_Transform3dJNI
 * Method:    log
 * Signature: (DDDDDDD)[D
 */
JNIEXPORT jdoubleArray JNICALL
Java_edu_wpi_first_math_jni_Transform3dJNI_log
  (JNIEnv* env, jclass, jdouble relX, jdouble relY, jdouble relZ, jdouble relQw,
   jdouble relQx, jdouble relQy, jdouble relQz)
{
  frc::Transform3d transform3d{
      relX * mp::m, relY * mp::m, relZ * mp::m,
      frc::Rotation3d{frc::Quaternion{relQw, relQx, relQy, relQz}}};

  frc::Twist3d result = transform3d.Log();

  return MakeJDoubleArray(env, {{mp::value(result.dx), mp::value(result.dy),
                                 mp::value(result.dz), mp::value(result.rx),
                                 mp::value(result.ry), mp::value(result.rz)}});
}

}  // extern "C"
