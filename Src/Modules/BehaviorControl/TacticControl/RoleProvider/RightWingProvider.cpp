#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h>
#include "RightWingProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Utils/PositionUtils.h"
#include "Utils/ThresholdUtils.h"

void RightWingProvider::update(RightWing& role)
{
  role.stopAtTarget = true;

  decide(role, theBallSymbols, theFieldDimensions, theGameInfo, theGameSymbols);
}

void RightWingProvider::stateReady_kickOff_own(RightWing& role, const Vector2f& ballPosition)
{
  ThresholdUtils::setThreshholdsHeigh(role);
  const float x = -theFieldDimensions.xPosOpponentPenaltyArea / 5;
  const float y = theFieldDimensions.yPosRightPenaltyArea;
  PositionUtils::setPosition(role, x, y);
  PositionUtils::turnToPosition(role, {700.f, 0.f});
}

void RightWingProvider::stateReady_kickOff_opponent(RightWing& role, const Vector2f& ballPosition)
{
  ThresholdUtils::setThreshholdsHeigh(role);
  const float x = -theFieldDimensions.xPosOpponentPenaltyArea / 2;
  const float y = theFieldDimensions.yPosRightPenaltyArea;
  PositionUtils::setPosition(role, x, y);
  PositionUtils::turnTowardsBall(role, theBallSymbols);
}

float RightWingProvider::goalKick_own(RightWing& role, bool left)
{
  regularPlay(role);
  return 0.f;
}

float RightWingProvider::goalKick_opponent(RightWing& role, bool left)
{
  regularPlay(role);
  return 0;
}

float RightWingProvider::pushingFreeKick_own(RightWing& role)
{
  regularPlay(role);
  return 0;
}

float RightWingProvider::pushingFreeKick_opponent(RightWing& role)
{
  regularPlay(role);
  return 0;
}

float RightWingProvider::cornerKick_own(RightWing& role, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(role);
  return 0;
}

float RightWingProvider::cornerKick_opponent(RightWing& role, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(role);
  return 0;
}

float RightWingProvider::kickIn_own(RightWing& role, bool left)
{
  regularPlay(role);
  return 0;
}

float RightWingProvider::kickIn_opponent(RightWing& role, bool left)
{
  regularPlay(role);
  return 0;
}

float RightWingProvider::stateReady_penaltyKick_own(RightWing& role)
{
  regularPlay(role);
  return 0;
}

float RightWingProvider::stateReady_penaltyKick_opponent(RightWing& role)
{
  regularPlay(role);
  return 0;
}

void RightWingProvider::regularPlay(RightWing& role)
{
  ThresholdUtils::setThreshholdsExtremeHeigh(role, theTacticSymbols.activity);

  PositionUtils::turnTowardsBall(role, theBallSymbols);

  float x = theBallSymbols.ballPositionField.x() + 1000.f - 2000.f * (float)theTacticSymbols.defensiveBehavior;
  x = MathUtils::clamp_f(x, theFieldDimensions.xPosOwnPenaltyArea * 2 / 3, theFieldDimensions.xPosOpponentPenaltyArea * 2 / 3);
  const float yOffset = theFieldDimensions.yPosLeftSideline / 5;
  const float y = theFieldDimensions.yPosRightSideline + yOffset + yOffset * (float)theTacticSymbols.defensiveBehavior;
  PositionUtils::setPosition(role, x, y);
}

MAKE_MODULE(RightWingProvider, behaviorControl)
