/**
 * @file Tools/Math/RotationMatrix.cpp
 * Implementation of class RotationMatrix
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author Max Risler
 * @author <a href="mailto:alexists@tzi.de">Alexis Tsogias</a>
 */

#include "Approx.h"
#include "BHMath.h"
#include "Rotation.h"
#include "RotationMatrix.h"
#include "Tools/Range.h"

RotationMatrix::RotationMatrix(const float& yaw, const float& pitch, const float& roll)
{
  const float cy = std::cos(yaw);
  const float sy = std::sin(yaw);
  const float cp = std::cos(pitch);
  const float sp = std::sin(pitch);
  const float cr = std::cos(roll);
  const float sr = std::sin(roll);

  Matrix3f& mat = *this;
  mat(0, 0) = cr*cp;
  mat(0, 1) = -sr*cy + cr*sp*sy;
  mat(0, 2) = sr*sy + cr*sp*cy;
  mat(1, 0) = sr*cp;
  mat(1, 1) = cr*cy + sr*sp*sy;
  mat(1, 2) = -cr*sy + sr*sp*cy;
  mat(2, 0) = -sp;
  mat(2, 1) = cp*sy;
  mat(2, 2) = cp*cy;
}

AngleAxisf RotationMatrix::getAngleAxis() const
{
  return AngleAxisf(*this);
}

Vector3f RotationMatrix::getPackedAngleAxis() const
{
  return Rotation::AngleAxis::pack(getAngleAxis());
}

Vector3f RotationMatrix::getPackedAngleAxisFaulty() const
{
  // This function suffers from a wrong handling of +-180° +- 1° rotations.
  // Additionally normalizing the result vector by (2.f * std::sin(angle)) is imprecise.
  // Unfortunately, the KickEngine kicks are tuned with this function.

  float cosine = (trace() - 1.f) * 0.5f;
  ASSERT(std::isfinite(cosine));
  if(cosine > 1.f || cosine < -1.f)
    cosine = 1.f;
  const float angle = std::acos(cosine);
  if(angle == 0.f)
    return Vector3f::Zero();
  else
  {
    const Matrix3f& mat = *this;
    Vector3f result(mat(2, 1) - mat(1, 2), mat(0, 2) - mat(2, 0), mat(1, 0) - mat(0, 1));
    result *= angle / (2.f * std::sin(angle));
    return result;
  }
}

RotationMatrix& RotationMatrix::rotateX(const float angle)
{
  Matrix3f& mat = *this;
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  const float m01 = mat(0, 1);
  const float m11 = mat(1, 1);
  const float m21 = mat(2, 1);
  mat(0, 1) = m01 * c + mat(0, 2) * s;
  mat(1, 1) = m11 * c + mat(1, 2) * s;
  mat(2, 1) = m21 * c + mat(2, 2) * s;
  mat(0, 2) = mat(0, 2) * c - m01 * s;
  mat(1, 2) = mat(1, 2) * c - m11 * s;
  mat(2, 2) = mat(2, 2) * c - m21 * s;
  return *this;
}

RotationMatrix& RotationMatrix::rotateY(const float angle)
{
  Matrix3f& mat = *this;
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  const float m00 = mat(0, 0);
  const float m10 = mat(1, 0);
  const float m20 = mat(2, 0);
  mat(0, 0) = m00 * c - mat(0, 2) * s;
  mat(1, 0) = m10 * c - mat(1, 2) * s;
  mat(2, 0) = m20 * c - mat(2, 2) * s;
  mat(0, 2) = mat(0, 2) * c + m00 * s;
  mat(1, 2) = mat(1, 2) * c + m10 * s;
  mat(2, 2) = mat(2, 2) * c + m20 * s;
  return *this;
}

RotationMatrix& RotationMatrix::rotateZ(const float angle)
{
  Matrix3f& mat = *this;
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  const float m00 = mat(0, 0);
  const float m10 = mat(1, 0);
  const float m20 = mat(2, 0);
  mat(0, 0) = m00 * c + mat(0, 1) * s;
  mat(1, 0) = m10 * c + mat(1, 1) * s;
  mat(2, 0) = m20 * c + mat(2, 1) * s;
  mat(0, 1) = mat(0, 1) * c - m00 * s;
  mat(1, 1) = mat(1, 1) * c - m10 * s;
  mat(2, 1) = mat(2, 1) * c - m20 * s;
  return *this;
}

float RotationMatrix::getXAngle() const
{
  const Matrix3f& mat = *this;
  const float h = std::sqrt(mat(1, 2) * mat(1, 2) + mat(2, 2) * mat(2, 2));
  if(Approx::isZero(h))
    return 0.f;
  else
    return std::acos(mat(2, 2) / h) * -sgnNeg(mat(1, 2));
}

float RotationMatrix::getYAngle() const
{
  const Matrix3f& mat = *this;
  const float h = std::sqrt(mat(0, 0) * mat(0, 0) + mat(2, 0) * mat(2, 0));
  if(Approx::isZero(h))
    return 0.f;
  else
    return std::acos(mat(0, 0) / h) * -sgnNeg(mat(2, 0));
}

float RotationMatrix::getZAngle() const
{
  const Matrix3f& mat = *this;
  const float h = std::sqrt(mat(0, 0) * mat(0, 0) + mat(1, 0) * mat(1, 0));
  if(Approx::isZero(h))
    return 0.f;
  else
    return std::acos(mat(0, 0) / h) * sgnPos(mat(1, 0));
}

RotationMatrix RotationMatrix::aroundX(const float angle)
{
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  return (RotationMatrix() <<
          1.f, 0.f, 0.f,
          0.f, c, -s,
          0.f, s, c).finished();
}

RotationMatrix RotationMatrix::aroundY(const float angle)
{
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  return (RotationMatrix() <<
          c, 0.f, s,
          0.f, 1.f, 0.f,
          -s, 0.f, c).finished();
}

RotationMatrix RotationMatrix::aroundZ(const float angle)
{
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  return (RotationMatrix() <<
          c, -s, 0.f,
          s, c, 0.f,
          0.f, 0.f, 1.f).finished();
}

RotationMatrix RotationMatrix::fromEulerAngles(const float x, const float y, const float z)
{
  return Rotation::Euler::fromAngles(x, y, z);
}

RotationMatrix RotationMatrix::fromEulerAngles(const Vector3f rotation)
{
  return Rotation::Euler::fromAngles(rotation);
}
