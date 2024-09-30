#pragma once
#include "Representations/Perception/RobotsPercept.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Compressed.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Pose3f.h"
#include <vector>

STREAMABLE(DirectionInfo,
  void draw(const Vector2f& ballPosition) const;
  int getIndex(const Angle& angle) const;

  ,

  (Angle) stepAngle,
  (std::vector<Angle>) angles,
  (std::vector<float>) blockedDistances,
  (std::vector<float>) intoGoalKickOutsideDistances,
  (std::vector<float>) intoKickInOutsideDistances,
  (std::vector<float>) intoCornerKickOutsideDistances,
  (std::vector<float>) intoOpponentGoalDistances,
  (std::vector<float>) intoOwnGoalDistances
);