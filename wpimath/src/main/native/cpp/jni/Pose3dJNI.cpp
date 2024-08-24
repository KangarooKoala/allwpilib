// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <jni.h>

#include <wpi/jni_util.h>

#include "edu_wpi_first_math_jni_Pose3dJNI.h"
#include "frc/geometry/Pose3d.h"

using namespace wpi::java;

extern "C" {

/*
 * Class:     edu_wpi_first_math_jni_Pose3dJNI
 * Method:    exp
 * Signature: (DDDDDDDDDDDDD)[D
 */
JNIEXPORT jdoubleArray JNICALL
Java_edu_wpi_first_math_jni_Pose3dJNI_exp
  (JNIEnv* env, jclass, jdouble poseX, jdouble poseY, jdouble poseZ,
   jdouble poseQw, jdouble poseQx, jdouble poseQy, jdouble poseQz,
   jdouble twistDx, jdouble twistDy, jdouble twistDz, jdouble twistRx,
   jdouble twistRy, jdouble twistRz)
{
  frc::Pose3d pose{
      poseX * units::meter, poseY * units::meter, poseZ * units::meter,
      frc::Rotation3d{frc::Quaternion{poseQw, poseQx, poseQy, poseQz}}};
  frc::Twist3d twist{twistDx * units::meter,  twistDy * units::meter,
                     twistDz * units::meter,  twistRx * units::radian,
                     twistRy * units::radian, twistRz * units::radian};

  frc::Pose3d result = pose.Exp(twist);

  const auto& resultQuaternion = result.Rotation().GetQuaternion();
  return MakeJDoubleArray(
      env, {{result.X().value(), result.Y().value(), result.Z().value(),
             resultQuaternion.W(), resultQuaternion.X(), resultQuaternion.Y(),
             resultQuaternion.Z()}});
}

/*
 * Class:     edu_wpi_first_math_jni_Pose3dJNI
 * Method:    log
 * Signature: (DDDDDDDDDDDDDD)[D
 */
JNIEXPORT jdoubleArray JNICALL
Java_edu_wpi_first_math_jni_Pose3dJNI_log
  (JNIEnv* env, jclass, jdouble startX, jdouble startY, jdouble startZ,
   jdouble startQw, jdouble startQx, jdouble startQy, jdouble startQz,
   jdouble endX, jdouble endY, jdouble endZ, jdouble endQw, jdouble endQx,
   jdouble endQy, jdouble endQz)
{
  frc::Pose3d startPose{
      startX * units::meter, startY * units::meter, startZ * units::meter,
      frc::Rotation3d{frc::Quaternion{startQw, startQx, startQy, startQz}}};
  frc::Pose3d endPose{
      endX * units::meter, endY * units::meter, endZ * units::meter,
      frc::Rotation3d{frc::Quaternion{endQw, endQx, endQy, endQz}}};

  frc::Twist3d result = startPose.Log(endPose);

  return MakeJDoubleArray(
      env, {{result.dx.value(), result.dy.value(), result.dz.value(),
             result.rx.value(), result.ry.value(), result.rz.value()}});
}

}  // extern "C"
