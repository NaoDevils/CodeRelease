#include "BackWingProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Utils/PositionUtils.h"
#include "Utils/ThresholdUtils.h"
#include "Utils/MathUtils.h"

void BackWingProvider::update(BackWing& role)
{
  role.stopAtTarget = true;

  decide(role, theBallSymbols, theFieldDimensions, theGameInfo, theGameSymbols);
}

void BackWingProvider::stateReady_kickOff_own(BackWing& role, const Vector2f& ballPosition)
{
  regularPlay(role);
}

void BackWingProvider::stateReady_kickOff_opponent(BackWing& role, const Vector2f& ballPosition)
{
  ThresholdUtils::setThreshholdsHeigh(role);
  const float x = -theFieldDimensions.xPosOpponentPenaltyArea / 1.2f;
  const float y = 500.f;
  PositionUtils::setPosition(role, x, y);
  PositionUtils::turnTowardsBall(role, theBallSymbols);
}

float BackWingProvider::goalKick_own(BackWing& role, bool left)
{
  regularPlay(role);
  return 0.f;
}

float BackWingProvider::goalKick_opponent(BackWing& role, bool left)
{
  regularPlay(role);
  return 0;
}

float BackWingProvider::pushingFreeKick_own(BackWing& role)
{
  regularPlay(role);
  return 0;
}

float BackWingProvider::pushingFreeKick_opponent(BackWing& role)
{
  regularPlay(role);
  return 0;
}

float BackWingProvider::cornerKick_own(BackWing& role, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(role);
  return 0;
}

float BackWingProvider::cornerKick_opponent(BackWing& role, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(role);
  return 0;
}

float BackWingProvider::kickIn_own(BackWing& role, bool left)
{
  regularPlay(role);
  return 0;
}

float BackWingProvider::kickIn_opponent(BackWing& role, bool left)
{
  regularPlay(role);
  return 0;
}

float BackWingProvider::stateReady_penaltyKick_own(BackWing& role)
{
  regularPlay(role);
  return 0;
}

float BackWingProvider::stateReady_penaltyKick_opponent(BackWing& role)
{
  bool robotIsLeft = (theRobotPose.translation.y() >= 0.f);
  float targetPosY = (robotIsLeft ? 1.f : -1.f) * (theFieldDimensions.yPosLeftPenaltyArea + 400.f);

  role.optPosition.translation.x() = theFieldDimensions.xPosOwnPenaltyArea - theFieldDimensions.xPosOwnPenaltyArea / 2;
  role.optPosition.translation.y() = targetPosY;
  PositionUtils::turnTowardsBall(role, theBallSymbols);

  role.thresholdRotation = 10_deg;
  role.thresholdXBack = 50.f;
  role.thresholdXFront = 100.f;
  role.thresholdY = 100.f;
  role.stopAtTarget = true;
  role.previewArrival = true;
  return 0;
}

void BackWingProvider::regularPlay(BackWing& role)
{
  ThresholdUtils::setThreshholdsExtremeHeigh(role, theTacticSymbols.activity);

  PositionUtils::turnTowardsBall(role, theBallSymbols);

  const float x = MathUtils::clamp_f(theBallSymbols.ballPositionField.x() - 2000.f, theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.xPosOpponentPenaltyArea / 2);
  const float y = 500.f;
  PositionUtils::setPosition(role, x, y);
}

MAKE_MODULE(BackWingProvider, behaviorControl)
