// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <jni.h>

#include <wpi/array.h>
#include <wpi/jni_util.h>

#include "edu_wpi_first_math_jni_Ellipse2dJNI.h"
#include "frc/geometry/Ellipse2d.h"
#include "frc/units.h"

using namespace wpi::java;

extern "C" {

/*
 * Class:     edu_wpi_first_math_jni_Ellipse2dJNI
 * Method:    nearest
 * Signature: (DDDDDDD[D)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_math_jni_Ellipse2dJNI_nearest
  (JNIEnv* env, jclass, jdouble centerX, jdouble centerY, jdouble centerHeading,
   jdouble xSemiAxis, jdouble ySemiAxis, jdouble pointX, jdouble pointY,
   jdoubleArray nearestPoint)
{
  auto point = frc::Ellipse2d{frc::Pose2d{centerX * mp::m, centerY * mp::m,
                                          centerHeading * mp::rad},
                              xSemiAxis * mp::m, ySemiAxis * mp::m}
                   .Nearest({pointX * mp::m, pointY * mp::m});

  wpi::array buf{mp::value(point.X()), mp::value(point.Y())};
  env->SetDoubleArrayRegion(nearestPoint, 0, 2, buf.data());
}

}  // extern "C"
