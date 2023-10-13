#pragma once

#include <vector>

#include "Tools/Math/Eigen.h"
#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/MotionControl/KickRequest.h"

STREAMABLE(DynPoint,
  DynPoint() = default;
  DynPoint(int limb, int phaseNumber, int duration, const Vector3f& translation, const Vector3f& angle, const Vector3f& odometryOffset);
  DynPoint(int limb, int phaseNumber, const Vector3f& translation);

  bool operator==(const DynPoint& other) const
  {
              return limb == other.limb &&
                  phaseNumber == other.phaseNumber &&
                  duration == other.duration &&
                  translation == other.translation &&
                  angle == other.angle &&
                  odometryOffset == other.odometryOffset;
  },

  (int) limb,
  (int) phaseNumber,
  (int)(0) duration,
  (Vector3f)(Vector3f::Zero()) translation,
  (Vector3f)(Vector3f::Zero()) angle,
  (Vector3f)(Vector3f::Zero()) odometryOffset
);

inline DynPoint::DynPoint(int limb, int phaseNumber, int duration, const Vector3f& translation, const Vector3f& angle, const Vector3f& odometryOffset)
    : limb(limb), phaseNumber(phaseNumber), duration(duration), translation(translation), angle(angle), odometryOffset(odometryOffset)
{
}

inline DynPoint::DynPoint(int limb, int phaseNumber, const Vector3f& translation) : limb(limb), phaseNumber(phaseNumber), translation(translation) {}