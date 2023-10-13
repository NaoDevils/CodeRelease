#include "FrontWingProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Utils/PositionUtils.h"
#include "Utils/ThresholdUtils.h"
#include "Utils/MathUtils.h"

void FrontWingProvider::update(FrontWing& role)
{
  role.stopAtTarget = true;

  decide(role, theBallSymbols, theFieldDimensions, theGameInfo, theGameSymbols);
}

void FrontWingProvider::stateReady_kickOff_own(FrontWing& role, const Vector2f& ballPosition)
{
  PositionUtils::setPosition(role, -500.f, 0);
  PositionUtils::turnToPosition(role, ballPosition);
  ThresholdUtils::setThreshholdsLow(role);
}

void FrontWingProvider::stateReady_kickOff_opponent(FrontWing& role, const Vector2f& ballPosition)
{
  role.optPosition.translation.x() = -theFieldDimensions.centerCircleRadius - theBehaviorConfiguration.behaviorParameters.kickOffLineDistance;
  role.optPosition.translation.y() = 0;
  role.optPosition.rotation = 0;
  role.thresholdXFront = 50;
  role.thresholdXBack = 50;
  role.thresholdY = 25;
}

float FrontWingProvider::goalKick_own(FrontWing& role, bool left)
{
  regularPlay(role);
  return 0.f;
}

float FrontWingProvider::goalKick_opponent(FrontWing& role, bool left)
{
  regularPlay(role);
  return 0;
}

float FrontWingProvider::pushingFreeKick_own(FrontWing& role)
{
  regularPlay(role);
  return 0;
}

float FrontWingProvider::pushingFreeKick_opponent(FrontWing& role)
{
  regularPlay(role);
  return 0;
}

float FrontWingProvider::cornerKick_own(FrontWing& role, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(role);
  return 0;
}

float FrontWingProvider::cornerKick_opponent(FrontWing& role, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(role);
  return 0;
}

float FrontWingProvider::kickIn_own(FrontWing& role, bool left)
{
  regularPlay(role);
  return 0;
}

float FrontWingProvider::kickIn_opponent(FrontWing& role, bool left)
{
  regularPlay(role);
  return 0;
}

float FrontWingProvider::stateReady_penaltyKick_own(FrontWing& role)
{
  regularPlay(role);
  return 0;
}

float FrontWingProvider::stateReady_penaltyKick_opponent(FrontWing& role)
{
  regularPlay(role);
  return 0;
}

void FrontWingProvider::regularPlay(FrontWing& role)
{
  ThresholdUtils::setThreshholdsExtremeHeigh(role, theTacticSymbols.activity);

  PositionUtils::turnTowardsBall(role, theBallSymbols);

  const float x = MathUtils::clamp_f(theBallSymbols.ballPositionField.x() + 2000.f, -theFieldDimensions.xPosOpponentPenaltyArea / 2, theFieldDimensions.xPosOpponentPenaltyArea);
  const float y = -500.f;
  PositionUtils::setPosition(role, x, y);
}

MAKE_MODULE(FrontWingProvider, behaviorControl)
