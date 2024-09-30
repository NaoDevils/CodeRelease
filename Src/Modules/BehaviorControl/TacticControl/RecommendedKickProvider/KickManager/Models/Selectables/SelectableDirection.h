#pragma once

#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h"
#include "Tools/Math/Geometry.h"

class SelectableDirection
{

public:
  SelectableDirection(const Vector2f& ballPosition,
      const Angle angle,
      const float intoBlockedDistance,
      const float intoGoalKickOutsideDistance,
      const float intoKickInOutsideDistance,
      const float intoCornerKickOutsideDistance,
      const float intoOpponentsGoalDistance,
      const float intoOwnGoalDistance)
      : ballPosition(ballPosition), angle(angle), intoBlockedDistance(intoBlockedDistance), intoGoalKickOutsideDistance(intoGoalKickOutsideDistance),
        intoKickInOutsideDistance(intoKickInOutsideDistance), intoCornerKickOutsideDistance(intoCornerKickOutsideDistance), intoOpponentsGoalDistance(intoOpponentsGoalDistance),
        intoOwnGoalDistance(intoOwnGoalDistance)
  {
    direction = MathUtils::angleToVector(angle);

    const float minToBorderDistance = std::max(
        intoGoalKickOutsideDistance, std::max(intoKickInOutsideDistance, std::max(intoCornerKickOutsideDistance, std::max(intoOpponentsGoalDistance, intoOwnGoalDistance))));

    if (intoBlockedDistance != -1 && intoBlockedDistance < minToBorderDistance)
    {
      // blocked
      isFirstIntoBlocked = true;
      isFirstIntoGoalKickOutside = false;
      isFirstIntoKickInOutside = false;
      isFirstIntoCornerKickOutside = false;
      isFirstIntoOpponentsGoal = false;
      isFirstIntoOwnGoal = false;
      isOutsideAnGoesFurtherOutside = false;
    }
    else
    {
      if (minToBorderDistance == -1)
      {
        // is outside and goes outside, everything is -1
        isFirstIntoBlocked = false;
        isFirstIntoGoalKickOutside = false;
        isFirstIntoKickInOutside = false;
        isFirstIntoCornerKickOutside = false;
        isFirstIntoOpponentsGoal = false;
        isFirstIntoOwnGoal = false;
        isOutsideAnGoesFurtherOutside = true;
      }
      else
      {
        isFirstIntoBlocked = false;
        isFirstIntoGoalKickOutside = minToBorderDistance == intoGoalKickOutsideDistance;
        isFirstIntoKickInOutside = minToBorderDistance == intoKickInOutsideDistance;
        isFirstIntoCornerKickOutside = minToBorderDistance == intoCornerKickOutsideDistance;
        isFirstIntoOpponentsGoal = minToBorderDistance == intoOpponentsGoalDistance;
        isFirstIntoOwnGoal = minToBorderDistance == intoOwnGoalDistance;
        isOutsideAnGoesFurtherOutside = false;
      }
    }

    ASSERT(1
        == int(isFirstIntoBlocked) + int(isFirstIntoGoalKickOutside) + int(isFirstIntoKickInOutside) + int(isFirstIntoCornerKickOutside) + int(isFirstIntoOpponentsGoal)
            + int(isFirstIntoOwnGoal) + int(isOutsideAnGoesFurtherOutside));
  }

  Vector2f ballPosition;

  Angle angle;
  Vector2f direction;

  // -1 if invalid
  float intoBlockedDistance;
  float intoGoalKickOutsideDistance;
  float intoKickInOutsideDistance;
  float intoCornerKickOutsideDistance;
  float intoOpponentsGoalDistance;
  float intoOwnGoalDistance;

  bool isFirstIntoBlocked;
  bool isFirstIntoGoalKickOutside;
  bool isFirstIntoKickInOutside;
  bool isFirstIntoCornerKickOutside;
  bool isFirstIntoOpponentsGoal;
  bool isFirstIntoOwnGoal;

  bool isOutsideAnGoesFurtherOutside;
};
