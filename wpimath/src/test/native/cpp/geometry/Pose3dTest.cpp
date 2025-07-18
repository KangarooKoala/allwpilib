// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include <gtest/gtest.h>
#include <wpi/array.h>

#include "frc/geometry/Pose3d.h"
#include "frc/units-usc.h"
#include "frc/units.h"

using namespace frc;

TEST(Pose3dTest, RotateBy) {
  constexpr auto x = 1.0 * mp::m;
  constexpr auto y = 2.0 * mp::m;
  const Pose3d initial{
      x, y, 0.0 * mp::m,
      Rotation3d{0.0 * mp::deg, 0.0 * mp::deg, 45.0 * mp::deg}};

  constexpr mp::quantity<mp::rad> yaw = 5.0 * mp::deg;
  const Rotation3d rotation{0.0 * mp::deg, 0.0 * mp::deg, yaw};
  const auto rotated = initial.RotateBy(rotation);

  // Translation is rotated by CCW rotation matrix
  double c = mp::cos(yaw);
  double s = mp::sin(yaw);
  EXPECT_DOUBLE_EQ(c * mp::value(x) - s * mp::value(y), mp::value(rotated.X()));
  EXPECT_DOUBLE_EQ(s * mp::value(x) + c * mp::value(y), mp::value(rotated.Y()));
  EXPECT_DOUBLE_EQ(0.0, mp::value(rotated.Z()));
  EXPECT_DOUBLE_EQ(0.0, mp::value(rotated.Rotation().X()));
  EXPECT_DOUBLE_EQ(0.0, mp::value(rotated.Rotation().Y()));
  EXPECT_DOUBLE_EQ(mp::value(initial.Rotation().Z()) + mp::value(rotation.Z()),
                   mp::value(rotated.Rotation().Z()));
}

TEST(Pose3dTest, TestTransformByRotations) {
  constexpr double kEpsilon = 1E-9;

  const Pose3d initialPose{
      0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m,
      Rotation3d{0.0 * mp::deg, 0.0 * mp::deg, 0.0 * mp::deg}};
  const Transform3d transform1{
      Translation3d{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m},
      Rotation3d{90.0 * mp::deg, 45.0 * mp::deg, 0.0 * mp::deg}};
  const Transform3d transform2{
      Translation3d{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m},
      Rotation3d{-90.0 * mp::deg, 0.0 * mp::deg, 0.0 * mp::deg}};
  const Transform3d transform3{
      Translation3d{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m},
      Rotation3d{0.0 * mp::deg, -45.0 * mp::deg, 0.0 * mp::deg}};

  Pose3d finalPose = initialPose.TransformBy(transform1)
                         .TransformBy(transform2)
                         .TransformBy(transform3);

  EXPECT_NEAR(mp::value(finalPose.Rotation().X()),
              mp::value(initialPose.Rotation().X()), kEpsilon);
  EXPECT_NEAR(mp::value(finalPose.Rotation().Y()),
              mp::value(initialPose.Rotation().Y()), kEpsilon);
  EXPECT_NEAR(mp::value(finalPose.Rotation().Z()),
              mp::value(initialPose.Rotation().Z()), kEpsilon);
}

TEST(Pose3dTest, TransformBy) {
  Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  const Pose3d initial{1.0 * mp::m, 2.0 * mp::m, 0.0 * mp::m,
                       Rotation3d{zAxis, 45.0 * mp::deg}};
  const Transform3d transform{
      Translation3d{5.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m},
      Rotation3d{zAxis, 5.0 * mp::deg}};

  const auto transformed = initial + transform;

  EXPECT_DOUBLE_EQ(1.0 + 5.0 / std::sqrt(2.0), mp::value(transformed.X()));
  EXPECT_DOUBLE_EQ(2.0 + 5.0 / std::sqrt(2.0), mp::value(transformed.Y()));
  EXPECT_DOUBLE_EQ(mp::value(transformed.Rotation().Z()),
                   mp::value((50.0 * mp::deg).in(mp::rad)));
}

TEST(Pose3dTest, RelativeTo) {
  Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  const Pose3d initial{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m,
                       Rotation3d{zAxis, 45.0 * mp::deg}};
  const Pose3d final{5.0 * mp::m, 5.0 * mp::m, 0.0 * mp::m,
                     Rotation3d{zAxis, 45.0 * mp::deg}};

  const auto finalRelativeToInitial = final.RelativeTo(initial);

  EXPECT_DOUBLE_EQ(5.0 * std::sqrt(2.0), mp::value(finalRelativeToInitial.X()));
  EXPECT_DOUBLE_EQ(0.0, mp::value(finalRelativeToInitial.Y()));
  EXPECT_NEAR(0.0, mp::value(finalRelativeToInitial.Rotation().Z()), 1e-9);
}

TEST(Pose3dTest, RotateAround) {
  const Pose3d initial{5.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m, Rotation3d{}};
  const Translation3d point{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m};

  const auto rotated = initial.RotateAround(
      point, Rotation3d{0.0 * mp::deg, 0.0 * mp::deg, 180.0 * mp::deg});

  EXPECT_NEAR(-5.0, mp::value(rotated.X()), 1e-9);
  EXPECT_NEAR(0.0, mp::value(rotated.Y()), 1e-9);
  EXPECT_NEAR(mp::value((180.0 * mp::deg).in(mp::rad)),
              mp::value(rotated.Rotation().Z()), 1e-9);
}

TEST(Pose3dTest, Equality) {
  Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  const Pose3d a{0.0 * mp::m, 5.0 * mp::m, 0.0 * mp::m,
                 Rotation3d{zAxis, 43.0 * mp::deg}};
  const Pose3d b{0.0 * mp::m, 5.0 * mp::m, 0.0 * mp::m,
                 Rotation3d{zAxis, 43.0 * mp::deg}};
  EXPECT_TRUE(a == b);
}

TEST(Pose3dTest, Inequality) {
  Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  const Pose3d a{0.0 * mp::m, 5.0 * mp::m, 0.0 * mp::m,
                 Rotation3d{zAxis, 43.0 * mp::deg}};
  const Pose3d b{0.0 * mp::m, 5.0 * mp::ft, 0.0 * mp::m,
                 Rotation3d{zAxis, 43.0 * mp::deg}};
  EXPECT_TRUE(a != b);
}

TEST(Pose3dTest, Minus) {
  Eigen::Vector3d zAxis{0.0, 0.0, 1.0};

  const Pose3d initial{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m,
                       Rotation3d{zAxis, 45.0 * mp::deg}};
  const Pose3d final{5.0 * mp::m, 5.0 * mp::m, 0.0 * mp::m,
                     Rotation3d{zAxis, 45.0 * mp::deg}};

  const auto transform = final - initial;

  EXPECT_DOUBLE_EQ(5.0 * std::sqrt(2.0), mp::value(transform.X()));
  EXPECT_DOUBLE_EQ(0.0, mp::value(transform.Y()));
  EXPECT_NEAR(0.0, mp::value(transform.Rotation().Z()), 1e-9);
}

TEST(Pose3dTest, ToMatrix) {
  Pose3d before{1.0 * mp::m, 2.0 * mp::m, 3.0 * mp::m,
                Rotation3d{10.0 * mp::deg, 20.0 * mp::deg, 30.0 * mp::deg}};
  Pose3d after{before.ToMatrix()};

  EXPECT_EQ(before, after);
}

TEST(Pose3dTest, ToPose2d) {
  Pose3d pose{1.0 * mp::m, 2.0 * mp::m, 3.0 * mp::m,
              Rotation3d{20.0 * mp::deg, 30.0 * mp::deg, 40.0 * mp::deg}};
  Pose2d expected{1.0 * mp::m, 2.0 * mp::m, 40.0 * mp::deg};

  EXPECT_EQ(expected, pose.ToPose2d());
}

TEST(Pose3dTest, ComplexTwists) {
  wpi::array<Pose3d, 5> initial_poses{
      Pose3d{0.698303 * mp::m, -0.959096 * mp::m, 0.271076 * mp::m,
             Rotation3d{Quaternion{0.86403, -0.076866, 0.147234, 0.475254}}},
      Pose3d{0.634892 * mp::m, -0.765209 * mp::m, 0.117543 * mp::m,
             Rotation3d{Quaternion{0.84987, -0.070829, 0.162097, 0.496415}}},
      Pose3d{0.584827 * mp::m, -0.590303 * mp::m, -0.02557 * mp::m,
             Rotation3d{Quaternion{0.832743, -0.041991, 0.202188, 0.513708}}},
      Pose3d{0.505038 * mp::m, -0.451479 * mp::m, -0.112835 * mp::m,
             Rotation3d{Quaternion{0.816515, -0.002673, 0.226182, 0.531166}}},
      Pose3d{0.428178 * mp::m, -0.329692 * mp::m, -0.189707 * mp::m,
             Rotation3d{Quaternion{0.807886, 0.029298, 0.257788, 0.529157}}},
  };

  wpi::array<Pose3d, 5> final_poses{
      Pose3d{-0.230448 * mp::m, -0.511957 * mp::m, 0.198406 * mp::m,
             Rotation3d{Quaternion{0.753984, 0.347016, 0.409105, 0.379106}}},
      Pose3d{-0.088932 * mp::m, -0.343253 * mp::m, 0.095018 * mp::m,
             Rotation3d{Quaternion{0.638738, 0.413016, 0.536281, 0.365833}}},
      Pose3d{-0.107908 * mp::m, -0.317552 * mp::m, 0.133946 * mp::m,
             Rotation3d{Quaternion{0.653444, 0.417069, 0.465505, 0.427046}}},
      Pose3d{-0.123383 * mp::m, -0.156411 * mp::m, -0.047435 * mp::m,
             Rotation3d{Quaternion{0.652983, 0.40644, 0.431566, 0.47135}}},
      Pose3d{-0.084654 * mp::m, -0.019305 * mp::m, -0.030022 * mp::m,
             Rotation3d{Quaternion{0.620243, 0.429104, 0.479384, 0.44873}}},
  };

  for (size_t i = 0; i < initial_poses.size(); i++) {
    auto start = initial_poses[i];
    auto end = final_poses[i];

    auto twist = (end - start).Log();
    auto start_exp = start + twist.Exp();

    auto eps = 1E-5;

    EXPECT_NEAR(mp::value(start_exp.X()), mp::value(end.X()), eps);
    EXPECT_NEAR(mp::value(start_exp.Y()), mp::value(end.Y()), eps);
    EXPECT_NEAR(mp::value(start_exp.Z()), mp::value(end.Z()), eps);
    EXPECT_NEAR(start_exp.Rotation().GetQuaternion().W(),
                end.Rotation().GetQuaternion().W(), eps);
    EXPECT_NEAR(start_exp.Rotation().GetQuaternion().X(),
                end.Rotation().GetQuaternion().X(), eps);
    EXPECT_NEAR(start_exp.Rotation().GetQuaternion().Y(),
                end.Rotation().GetQuaternion().Y(), eps);
    EXPECT_NEAR(start_exp.Rotation().GetQuaternion().Z(),
                end.Rotation().GetQuaternion().Z(), eps);
  }
}

TEST(Pose3dTest, TwistNaN) {
  wpi::array<Pose3d, 2> initial_poses{
      Pose3d{6.32 * mp::m, 4.12 * mp::m, 0.00 * mp::m,
             Rotation3d{Quaternion{-0.9999999999999999, 0.0, 0.0,
                                   1.9208309264993548E-8}}},
      Pose3d{3.75 * mp::m, 2.95 * mp::m, 0.00 * mp::m,
             Rotation3d{Quaternion{0.9999999999999793, 0.0, 0.0,
                                   2.0352360299846772E-7}}},
  };

  wpi::array<Pose3d, 2> final_poses{
      Pose3d{6.33 * mp::m, 4.15 * mp::m, 0.00 * mp::m,
             Rotation3d{Quaternion{-0.9999999999999999, 0.0, 0.0,
                                   2.416890209039172E-8}}},
      Pose3d{3.66 * mp::m, 2.93 * mp::m, 0.00 * mp::m,
             Rotation3d{Quaternion{0.9999999999999782, 0.0, 0.0,
                                   2.0859477994905617E-7}}},
  };

  for (size_t i = 0; i < initial_poses.size(); i++) {
    auto start = initial_poses[i];
    auto end = final_poses[i];
    auto twist = (end - start).Log();

    EXPECT_FALSE(std::isnan(mp::value(twist.dx)));
    EXPECT_FALSE(std::isnan(mp::value(twist.dy)));
    EXPECT_FALSE(std::isnan(mp::value(twist.dz)));
    EXPECT_FALSE(std::isnan(mp::value(twist.rx)));
    EXPECT_FALSE(std::isnan(mp::value(twist.ry)));
    EXPECT_FALSE(std::isnan(mp::value(twist.rz)));
  }
}

TEST(Pose3dTest, Nearest) {
  const Pose3d origin{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m, Rotation3d{}};

  // Distance sort
  // poses are in order of closest to farthest away from the origin at
  // various positions in 3D space.
  const Pose3d pose1{1.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m, Rotation3d{}};
  const Pose3d pose2{0.0 * mp::m, 2.0 * mp::m, 0.0 * mp::m, Rotation3d{}};
  const Pose3d pose3{0.0 * mp::m, 0.0 * mp::m, 3.0 * mp::m, Rotation3d{}};
  const Pose3d pose4{2.0 * mp::m, 2.0 * mp::m, 2.0 * mp::m, Rotation3d{}};
  const Pose3d pose5{3.0 * mp::m, 3.0 * mp::m, 3.0 * mp::m, Rotation3d{}};

  EXPECT_DOUBLE_EQ(mp::value(pose3.X()),
                   mp::value(origin.Nearest({pose5, pose3, pose4}).X()));
  EXPECT_DOUBLE_EQ(mp::value(pose3.Y()),
                   mp::value(origin.Nearest({pose5, pose3, pose4}).Y()));
  EXPECT_DOUBLE_EQ(mp::value(pose3.Z()),
                   mp::value(origin.Nearest({pose5, pose3, pose4}).Z()));

  EXPECT_DOUBLE_EQ(mp::value(pose1.X()),
                   mp::value(origin.Nearest({pose1, pose2, pose3}).X()));
  EXPECT_DOUBLE_EQ(mp::value(pose1.Y()),
                   mp::value(origin.Nearest({pose1, pose2, pose3}).Y()));
  EXPECT_DOUBLE_EQ(mp::value(pose1.Z()),
                   mp::value(origin.Nearest({pose1, pose2, pose3}).Z()));

  EXPECT_DOUBLE_EQ(mp::value(pose2.X()),
                   mp::value(origin.Nearest({pose4, pose2, pose3}).X()));
  EXPECT_DOUBLE_EQ(mp::value(pose2.Y()),
                   mp::value(origin.Nearest({pose4, pose2, pose3}).Y()));
  EXPECT_DOUBLE_EQ(mp::value(pose2.Z()),
                   mp::value(origin.Nearest({pose4, pose2, pose3}).Z()));

  // Rotation component sort (when distance is the same)
  // Use the same translation to avoid distance differences
  const Translation3d translation{1.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m};

  const Pose3d poseA{translation, Rotation3d{}};  // No rotation
  const Pose3d poseB{translation,
                     Rotation3d{30.0 * mp::deg, 0.0 * mp::deg, 0.0 * mp::deg}};
  const Pose3d poseC{translation,
                     Rotation3d{0.0 * mp::deg, 45.0 * mp::deg, 0.0 * mp::deg}};
  const Pose3d poseD{translation,
                     Rotation3d{0.0 * mp::deg, 0.0 * mp::deg, 90.0 * mp::deg}};
  const Pose3d poseE{translation,
                     Rotation3d{180.0 * mp::deg, 0.0 * mp::deg, 0.0 * mp::deg}};

  auto result1 =
      Pose3d{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m, Rotation3d{}}.Nearest(
          {poseA, poseB, poseD});
  EXPECT_DOUBLE_EQ(mp::value(poseA.Rotation().X()),
                   mp::value(result1.Rotation().X()));
  EXPECT_DOUBLE_EQ(mp::value(poseA.Rotation().Y()),
                   mp::value(result1.Rotation().Y()));
  EXPECT_DOUBLE_EQ(mp::value(poseA.Rotation().Z()),
                   mp::value(result1.Rotation().Z()));

  auto result2 =
      Pose3d{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m,
             Rotation3d{25.0 * mp::deg, 0.0 * mp::deg, 0.0 * mp::deg}}
          .Nearest({poseB, poseC, poseD});
  EXPECT_DOUBLE_EQ(mp::value(poseB.Rotation().X()),
                   mp::value(result2.Rotation().X()));
  EXPECT_DOUBLE_EQ(mp::value(poseB.Rotation().Y()),
                   mp::value(result2.Rotation().Y()));
  EXPECT_DOUBLE_EQ(mp::value(poseB.Rotation().Z()),
                   mp::value(result2.Rotation().Z()));

  auto result3 =
      Pose3d{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m,
             Rotation3d{0.0 * mp::deg, 50.0 * mp::deg, 0.0 * mp::deg}}
          .Nearest({poseB, poseC, poseD});
  EXPECT_DOUBLE_EQ(mp::value(poseC.Rotation().X()),
                   mp::value(result3.Rotation().X()));
  EXPECT_DOUBLE_EQ(mp::value(poseC.Rotation().Y()),
                   mp::value(result3.Rotation().Y()));
  EXPECT_DOUBLE_EQ(mp::value(poseC.Rotation().Z()),
                   mp::value(result3.Rotation().Z()));

  auto result4 =
      Pose3d{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m,
             Rotation3d{0.0 * mp::deg, 0.0 * mp::deg, 85.0 * mp::deg}}
          .Nearest({poseA, poseC, poseD});
  EXPECT_DOUBLE_EQ(mp::value(poseD.Rotation().X()),
                   mp::value(result4.Rotation().X()));
  EXPECT_DOUBLE_EQ(mp::value(poseD.Rotation().Y()),
                   mp::value(result4.Rotation().Y()));
  EXPECT_DOUBLE_EQ(mp::value(poseD.Rotation().Z()),
                   mp::value(result4.Rotation().Z()));

  auto result5 =
      Pose3d{0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m,
             Rotation3d{170.0 * mp::deg, 0.0 * mp::deg, 0.0 * mp::deg}}
          .Nearest({poseA, poseD, poseE});
  EXPECT_DOUBLE_EQ(mp::value(poseE.Rotation().X()),
                   mp::value(result5.Rotation().X()));
  EXPECT_DOUBLE_EQ(mp::value(poseE.Rotation().Y()),
                   mp::value(result5.Rotation().Y()));
  EXPECT_DOUBLE_EQ(mp::value(poseE.Rotation().Z()),
                   mp::value(result5.Rotation().Z()));

  // Test with complex 3D rotations (combining roll, pitch, yaw)
  const Pose3d complexPose1{
      translation, Rotation3d{45.0 * mp::deg, 30.0 * mp::deg, 60.0 * mp::deg}};
  const Pose3d complexPose2{
      translation, Rotation3d{90.0 * mp::deg, 45.0 * mp::deg, 90.0 * mp::deg}};
  const Pose3d complexPose3{
      translation, Rotation3d{10.0 * mp::deg, 15.0 * mp::deg, 20.0 * mp::deg}};

  auto complexResult = Pose3d{
      0.0 * mp::m, 0.0 * mp::m, 0.0 * mp::m,
      Rotation3d{
          5.0 * mp::deg, 10.0 * mp::deg,
          15.0 * mp::deg}}.Nearest({complexPose1, complexPose2, complexPose3});
  EXPECT_DOUBLE_EQ(mp::value(complexPose3.Rotation().X()),
                   mp::value(complexResult.Rotation().X()));
  EXPECT_DOUBLE_EQ(mp::value(complexPose3.Rotation().Y()),
                   mp::value(complexResult.Rotation().Y()));
  EXPECT_DOUBLE_EQ(mp::value(complexPose3.Rotation().Z()),
                   mp::value(complexResult.Rotation().Z()));
}
