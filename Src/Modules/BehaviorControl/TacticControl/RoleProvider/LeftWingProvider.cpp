#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h>
#include "LeftWingProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Utils/PositionUtils.h"
#include "Utils/ThresholdUtils.h"

void LeftWingProvider::update(LeftWing& role)
{
  role.stopAtTarget = true;

  decide(role, theBallSymbols, theFieldDimensions, theGameInfo, theGameSymbols);
}

void LeftWingProvider::stateReady_kickOff_own(LeftWing& role, const Vector2f& ballPosition)
{
  ThresholdUtils::setThreshholdsHeigh(role);
  const float x = -theFieldDimensions.xPosOpponentPenaltyArea / 5;
  const float y = theFieldDimensions.yPosLeftPenaltyArea;
  PositionUtils::setPosition(role, x, y);
  PositionUtils::turnToPosition(role, {700.f, 0.f});
}

void LeftWingProvider::stateReady_kickOff_opponent(LeftWing& role, const Vector2f& ballPosition)
{
  ThresholdUtils::setThreshholdsHeigh(role);
  const float x = -theFieldDimensions.xPosOpponentPenaltyArea / 2;
  const float y = theFieldDimensions.yPosLeftPenaltyArea;
  PositionUtils::setPosition(role, x, y);
  PositionUtils::turnTowardsBall(role, theBallSymbols);
}

float LeftWingProvider::goalKick_own(LeftWing& role, bool left)
{
  regularPlay(role);
  return 0.f;
}

float LeftWingProvider::goalKick_opponent(LeftWing& role, bool left)
{
  regularPlay(role);
  return 0;
}

float LeftWingProvider::pushingFreeKick_own(LeftWing& role)
{
  regularPlay(role);
  return 0;
}

float LeftWingProvider::pushingFreeKick_opponent(LeftWing& role)
{
  regularPlay(role);
  return 0;
}

float LeftWingProvider::cornerKick_own(LeftWing& role, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(role);
  return 0;
}

float LeftWingProvider::cornerKick_opponent(LeftWing& role, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(role);
  return 0;
}

float LeftWingProvider::kickIn_own(LeftWing& role, bool left)
{
  regularPlay(role);
  return 0;
}

float LeftWingProvider::kickIn_opponent(LeftWing& role, bool left)
{
  regularPlay(role);
  return 0;
}

float LeftWingProvider::stateReady_penaltyKick_own(LeftWing& role)
{
  regularPlay(role);
  return 0;
}

float LeftWingProvider::stateReady_penaltyKick_opponent(LeftWing& role)
{
  regularPlay(role);
  return 0;
}

void LeftWingProvider::regularPlay(LeftWing& role)
{
  ThresholdUtils::setThreshholdsExtremeHeigh(role, theTacticSymbols.activity);

  PositionUtils::turnTowardsBall(role, theBallSymbols);

  float x = theBallSymbols.ballPositionField.x() + 1000.f - 2000.f * (float)theTacticSymbols.defensiveBehavior;
  x = MathUtils::clamp_f(x, theFieldDimensions.xPosOwnPenaltyArea * 2 / 3, theFieldDimensions.xPosOpponentPenaltyArea * 2 / 3);
  const float yOffset = theFieldDimensions.yPosLeftSideline / 5;
  const float y = theFieldDimensions.yPosLeftSideline - yOffset - yOffset * (float)theTacticSymbols.defensiveBehavior;
  PositionUtils::setPosition(role, x, y);
}

MAKE_MODULE(LeftWingProvider, behaviorControl)
