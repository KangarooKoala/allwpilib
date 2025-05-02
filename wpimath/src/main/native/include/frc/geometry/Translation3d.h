// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <initializer_list>
#include <span>

#include <Eigen/Core>
#include <wpi/SymbolExports.h>
#include <wpi/json_fwd.h>

#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/units.h"

namespace frc {

/**
 * Represents a translation in 3D space.
 * This object can be used to represent a point or a vector.
 *
 * This assumes that you are using conventional mathematical axes. When the
 * robot is at the origin facing in the positive X direction, forward is
 * positive X, left is positive Y, and up is positive Z.
 */
class WPILIB_DLLEXPORT Translation3d {
 public:
  /**
   * Constructs a Translation3d with X, Y, and Z components equal to zero.
   */
  constexpr Translation3d() = default;

  /**
   * Constructs a Translation3d with the X, Y, and Z components equal to the
   * provided values.
   *
   * @param x The x component of the translation.
   * @param y The y component of the translation.
   * @param z The z component of the translation.
   */
  constexpr Translation3d(mp::quantity<mp::m> x, mp::quantity<mp::m> y,
                          mp::quantity<mp::m> z)
      : m_x{x}, m_y{y}, m_z{z} {}

  /**
   * Constructs a Translation3d with the provided distance and angle. This is
   * essentially converting from polar coordinates to Cartesian coordinates.
   *
   * @param distance The distance from the origin to the end of the translation.
   * @param angle The angle between the x-axis and the translation vector.
   */
  constexpr Translation3d(mp::quantity<mp::m> distance,
                          const Rotation3d& angle) {
    auto rectangular =
        Translation3d{distance, 0.0 * mp::m, 0.0 * mp::m}.RotateBy(angle);
    m_x = rectangular.X();
    m_y = rectangular.Y();
    m_z = rectangular.Z();
  }

  /**
   * Constructs a Translation3d from a 3D translation vector. The values are
   * assumed to be in meters.
   *
   * @param vector The translation vector.
   */
  constexpr explicit Translation3d(const Eigen::Vector3d& vector)
      : m_x{vector.x() * mp::m},
        m_y{vector.y() * mp::m},
        m_z{vector.z() * mp::m} {}

  /**
   * Constructs a 3D translation from a 2D translation in the X-Y plane.
   *
   * @param translation The 2D translation.
   * @see Pose3d(Pose2d)
   * @see Transform3d(Transform2d)
   */
  constexpr explicit Translation3d(const Translation2d& translation)
      : Translation3d{translation.X(), translation.Y(), 0.0 * mp::m} {}

  /**
   * Calculates the distance between two translations in 3D space.
   *
   * The distance between translations is defined as
   * √((x₂−x₁)²+(y₂−y₁)²+(z₂−z₁)²).
   *
   * @param other The translation to compute the distance to.
   *
   * @return The distance between the two translations.
   */
  constexpr mp::quantity<mp::m> Distance(const Translation3d& other) const {
    return mp::sqrt(mp::pow<2>(other.m_x - m_x) + mp::pow<2>(other.m_y - m_y) +
                    mp::pow<2>(other.m_z - m_z));
  }

  /**
   * Calculates the squared distance between two translations in 3D space. This
   * is equivalent to squaring the result of Distance(Translation3d), but avoids
   * computing a square root.
   *
   * The squared distance between translations is defined as
   * (x₂−x₁)²+(y₂−y₁)²+(z₂−z₁)².
   *
   * @param other The translation to compute the squared distance to.
   * @return The squared distance between the two translations.
   */
  constexpr mp::quantity<mp::m2> SquaredDistance(
      const Translation3d& other) const {
    return mp::pow<2>(other.m_x - m_x) + mp::pow<2>(other.m_y - m_y) +
           mp::pow<2>(other.m_z - m_z);
  }

  /**
   * Returns the X component of the translation.
   *
   * @return The Z component of the translation.
   */
  constexpr mp::quantity<mp::m> X() const { return m_x; }

  /**
   * Returns the Y component of the translation.
   *
   * @return The Y component of the translation.
   */
  constexpr mp::quantity<mp::m> Y() const { return m_y; }

  /**
   * Returns the Z component of the translation.
   *
   * @return The Z component of the translation.
   */
  constexpr mp::quantity<mp::m> Z() const { return m_z; }

  /**
   * Returns a 3D translation vector representation of this translation.
   *
   * @return A 3D translation vector representation of this translation.
   */
  constexpr Eigen::Vector3d ToVector() const {
    return Eigen::Vector3d{{mp::value(m_x), mp::value(m_y), mp::value(m_z)}};
  }

  /**
   * Returns the norm, or distance from the origin to the translation.
   *
   * @return The norm of the translation.
   */
  constexpr mp::quantity<mp::m> Norm() const {
    return mp::sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
  }

  /**
   * Returns the squared norm, or squared distance from the origin to the
   * translation. This is equivalent to squaring the result of Norm(), but
   * avoids computing a square root.
   *
   * @return The squared norm of the translation.
   */
  constexpr mp::quantity<mp::m2> SquaredNorm() const {
    return m_x * m_x + m_y * m_y + m_z * m_z;
  }

  /**
   * Applies a rotation to the translation in 3D space.
   *
   * For example, rotating a Translation3d of &lt;2, 0, 0&gt; by 90 degrees
   * around the Z axis will return a Translation3d of &lt;0, 2, 0&gt;.
   *
   * @param other The rotation to rotate the translation by.
   *
   * @return The new rotated translation.
   */
  constexpr Translation3d RotateBy(const Rotation3d& other) const {
    Quaternion p{0.0, mp::value(m_x), mp::value(m_y), mp::value(m_z)};
    auto qprime = other.GetQuaternion() * p * other.GetQuaternion().Inverse();
    return Translation3d{qprime.X() * mp::m, qprime.Y() * mp::m,
                         qprime.Z() * mp::m};
  }

  /**
   * Rotates this translation around another translation in 3D space.
   *
   * @param other The other translation to rotate around.
   * @param rot The rotation to rotate the translation by.
   * @return The new rotated translation.
   */
  constexpr Translation3d RotateAround(const Translation3d& other,
                                       const Rotation3d& rot) const {
    return (*this - other).RotateBy(rot) + other;
  }

  /**
   * Computes the dot product between this translation and another translation
   * in 3D space.
   *
   * The dot product between two translations is defined as x₁x₂+y₁y₂+z₁z₂.
   *
   * @param other The translation to compute the dot product with.
   * @return The dot product between the two translations.
   */
  constexpr mp::quantity<mp::m2> Dot(const Translation3d& other) const {
    return m_x * other.X() + m_y * other.Y() + m_z * other.Z();
  }

  /**
   * Computes the cross product between this translation and another
   * translation in 3D space. The resulting translation will be perpendicular to
   * both translations.
   *
   * The 3D cross product between two translations is defined as <y₁z₂-y₂z₁,
   * z₁x₂-z₂x₁, x₁y₂-x₂y₁>.
   *
   * @param other The translation to compute the cross product with.
   * @return The cross product between the two translations.
   */
  constexpr Eigen::Vector<mp::quantity<mp::m2>, 3> Cross(
      const Translation3d& other) const {
    return Eigen::Vector<mp::quantity<mp::m2>, 3>{
        {m_y * other.Z() - other.Y() * m_z},
        {m_z * other.X() - other.Z() * m_x},
        {m_x * other.Y() - other.X() * m_y}};
  }

  /**
   * Returns a Translation2d representing this Translation3d projected into the
   * X-Y plane.
   */
  constexpr Translation2d ToTranslation2d() const {
    return Translation2d{m_x, m_y};
  }

  /**
   * Returns the sum of two translations in 3D space.
   *
   * For example, Translation3d{1.0, 2.5, 3.5} + Translation3d{2.0, 5.5, 7.5} =
   * Translation3d{3.0, 8.0, 11.0}.
   *
   * @param other The translation to add.
   *
   * @return The sum of the translations.
   */
  constexpr Translation3d operator+(const Translation3d& other) const {
    return {X() + other.X(), Y() + other.Y(), Z() + other.Z()};
  }

  /**
   * Returns the difference between two translations.
   *
   * For example, Translation3d{5.0, 4.0, 3.0} - Translation3d{1.0, 2.0, 3.0} =
   * Translation3d{4.0, 2.0, 0.0}.
   *
   * @param other The translation to subtract.
   *
   * @return The difference between the two translations.
   */
  constexpr Translation3d operator-(const Translation3d& other) const {
    return operator+(-other);
  }

  /**
   * Returns the inverse of the current translation. This is equivalent to
   * negating all components of the translation.
   *
   * @return The inverse of the current translation.
   */
  constexpr Translation3d operator-() const { return {-m_x, -m_y, -m_z}; }

  /**
   * Returns the translation multiplied by a scalar.
   *
   * For example, Translation3d{2.0, 2.5, 4.5} * 2 = Translation3d{4.0, 5.0,
   * 9.0}.
   *
   * @param scalar The scalar to multiply by.
   *
   * @return The scaled translation.
   */
  constexpr Translation3d operator*(double scalar) const {
    return {scalar * m_x, scalar * m_y, scalar * m_z};
  }

  /**
   * Returns the translation divided by a scalar.
   *
   * For example, Translation3d{2.0, 2.5, 4.5} / 2 = Translation3d{1.0, 1.25,
   * 2.25}.
   *
   * @param scalar The scalar to divide by.
   *
   * @return The scaled translation.
   */
  constexpr Translation3d operator/(double scalar) const {
    return operator*(1.0 / scalar);
  }

  /**
   * Checks equality between this Translation3d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  constexpr bool operator==(const Translation3d& other) const {
    return mp::abs(m_x - other.m_x) < 1E-9 * mp::m &&
           mp::abs(m_y - other.m_y) < 1E-9 * mp::m &&
           mp::abs(m_z - other.m_z) < 1E-9 * mp::m;
  }

  /**
   * Returns the nearest Translation3d from a collection of translations
   * @param translations The collection of translations.
   * @return The nearest Translation3d from the collection.
   */
  constexpr Translation3d Nearest(
      std::span<const Translation3d> translations) const {
    return *std::min_element(
        translations.begin(), translations.end(),
        [this](const Translation3d& a, const Translation3d& b) {
          return this->Distance(a) < this->Distance(b);
        });
  }

  /**
   * Returns the nearest Translation3d from a collection of translations
   * @param translations The collection of translations.
   * @return The nearest Translation3d from the collection.
   */
  constexpr Translation3d Nearest(
      std::initializer_list<Translation3d> translations) const {
    return *std::min_element(
        translations.begin(), translations.end(),
        [this](const Translation3d& a, const Translation3d& b) {
          return this->Distance(a) < this->Distance(b);
        });
  }

 private:
  mp::quantity<mp::m> m_x = 0.0 * mp::m;
  mp::quantity<mp::m> m_y = 0.0 * mp::m;
  mp::quantity<mp::m> m_z = 0.0 * mp::m;
};

WPILIB_DLLEXPORT
void to_json(wpi::json& json, const Translation3d& state);

WPILIB_DLLEXPORT
void from_json(const wpi::json& json, Translation3d& state);

}  // namespace frc

#include "frc/geometry/proto/Translation3dProto.h"
#include "frc/geometry/struct/Translation3dStruct.h"
