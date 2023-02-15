#pragma once

#include "Representations/BehaviorControl/RoleSymbols/Center.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/BehaviorControl/BallSymbols.h"

class BallUtils
{
public:
  static Vector2f getBallPosition(bool predict, const BallSymbols& theBallSymbols, const FrameInfo& theFrameInfo)
  {
    Vector2f ballPosition = predict ? theBallSymbols.ballPositionFieldPredicted : theBallSymbols.ballPositionField;
    if (theBallSymbols.timeSinceLastSeenByTeam == static_cast<int>(theFrameInfo.time)) // never seen -> assume center position
      ballPosition = Vector2f::Zero();
    return ballPosition;
  }
};
