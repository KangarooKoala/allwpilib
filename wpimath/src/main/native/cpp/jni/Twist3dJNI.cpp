// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <jni.h>

#include <wpi/jni_util.h>

#include "edu_wpi_first_math_jni_Twist3dJNI.h"
#include "frc/geometry/Transform3d.h"
#include "frc/geometry/Twist3d.h"
#include "frc/units.h"

using namespace wpi::java;

extern "C" {

/*
 * Class:     edu_wpi_first_math_jni_Twist3dJNI
 * Method:    exp
 * Signature: (DDDDDD)[D
 */
JNIEXPORT jdoubleArray JNICALL
Java_edu_wpi_first_math_jni_Twist3dJNI_exp
  (JNIEnv* env, jclass, jdouble twistDx, jdouble twistDy, jdouble twistDz,
   jdouble twistRx, jdouble twistRy, jdouble twistRz)
{
  frc::Twist3d twist{twistDx * mp::m,   twistDy * mp::m,   twistDz * mp::m,
                     twistRx * mp::rad, twistRy * mp::rad, twistRz * mp::rad};

  frc::Transform3d result = twist.Exp();

  const auto& resultQuaternion = result.Rotation().GetQuaternion();
  return MakeJDoubleArray(
      env, {{mp::value(result.X()), mp::value(result.Y()),
             mp::value(result.Z()), resultQuaternion.W(), resultQuaternion.X(),
             resultQuaternion.Y(), resultQuaternion.Z()}});
}

}  // extern "C"
