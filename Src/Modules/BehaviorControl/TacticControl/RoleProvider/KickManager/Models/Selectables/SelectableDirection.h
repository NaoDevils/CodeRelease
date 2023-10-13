#pragma once

#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h"
#include "Tools/Math/Geometry.h"
#include <functional>
#include <optional>
#include <utility>
#include <vector>

class SelectableDirection
{

public:
  SelectableDirection(const Vector2f& ballPosition, const Angle angle, const float distance, const bool distanceBlocked, const bool distanceOutside)
      : ballPosition(ballPosition), angle(angle), distance(distance), distanceBlocked(distanceBlocked), distanceOutside(distanceOutside)
  {
    ASSERT(!(distanceBlocked && distanceOutside));
    direction = MathUtils::angleToVector(angle);
  }

  Vector2f ballPosition;

  Angle angle;
  Vector2f direction;

  float distance;

  bool distanceBlocked;
  bool distanceOutside;

  float score;
};
