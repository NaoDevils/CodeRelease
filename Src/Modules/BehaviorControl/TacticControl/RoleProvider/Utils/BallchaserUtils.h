#pragma once

#include "Representations/BehaviorControl/RoleSymbols/Center.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "FieldUtils.h"
#include "HysterUtils.h"

class BallchaserUtils
{
public:
  static Vector2f getWaitPosition(const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions)
  {
    const Vector2f ownGoalCenter = FieldUtils::getOwnGoalCenter(theFieldDimensions);
    float y = ballPosition.y();
    if (y >= theFieldDimensions.yPosLeftGoal)
    {
      const float diff = y - theFieldDimensions.yPosLeftGoal;
      y = std::max(theFieldDimensions.yPosLeftGoal - diff, ownGoalCenter.y());
    }
    else if (theFieldDimensions.yPosRightGoal >= y)
    {
      const float diff = theFieldDimensions.yPosRightGoal - y;
      y = std::min(theFieldDimensions.yPosRightGoal + diff, ownGoalCenter.y());
    }
    Vector2f ownGoalPoint = {theFieldDimensions.xPosOwnGoal, y};
    return ballPosition + (ownGoalPoint - ballPosition).normalized() * 200.f /*todo constants*/;
  }
};
