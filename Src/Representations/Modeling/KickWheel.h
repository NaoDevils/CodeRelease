#pragma once
#include "Representations/Perception/RobotsPercept.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Compressed.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Pose3f.h"
#include <vector>

class DistanceInfo // Because the macro cant handle std::tuple
{
public:
  DistanceInfo(const float distance, const bool distanceBlocked, const bool distanceOutside)
      : distance(distance), distanceBlocked(distanceBlocked), distanceOutside(distanceOutside)
  {
  }
  float distance;
  bool distanceBlocked;
  bool distanceOutside;
};

STREAMABLE(KickWheel,
  void draw(const Vector2f& ballPosition) const;
  DistanceInfo getDistance(const Angle& angle, const bool hysteresis) const;

  ,

  (std::vector<Angle>) angles,
  (std::vector<float>) blockedDistances,
  (std::vector<float>) outsideDistances
);