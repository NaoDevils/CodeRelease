#pragma once

#include "Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/Kick.h"
#include "Representations/Modeling/RobotMap.h"

class BlockUtils
{
public:
  static bool isKickBlocked(const Vector2f& ballPosition, const Vector2f& targetPosition, float minKickWidth, bool hysteresis, const FieldDimensions& theFieldDimensions, const RobotMap& theRobotMap);

  static bool isTargetControlledByOpponent(const Vector2f& ballPosition, const Vector2f& targetPosition, float minFreeAroundTarget, bool hysteresis, const RobotMap& theRobotMap);

  /**
   * @brief brief Calculates for all opponent robots that are kind of close to opponent goal, if they block a goal kick.
  */
  static bool isKickBlockedOld(
      const Vector2f& ballPosition, const Vector2f& targetPosition, float minOpponentToBallDistance, float minOpeningAngleForDirectKick, float robotRadius, const RobotMap& theRobotMap);
};