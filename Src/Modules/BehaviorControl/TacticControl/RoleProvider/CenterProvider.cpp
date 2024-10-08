/**
* @file CenterProvider.cpp
*
* Implementation of class CenterProvider.
*
*/

#include "CenterProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Utils/PositionUtils.h"
#include "Utils/ThresholdUtils.h"
#include "Utils/MathUtils.h"

void CenterProvider::update(Center& role)
{
  role.stopAtTarget = true;

  decide(role, theBallSymbols, theFieldDimensions, theGameInfo, theGameSymbols);
}

void CenterProvider::stateReady_kickOff_own(Center& role, const Vector2f& ballPosition)
{
  regularPlay(role);
}

void CenterProvider::stateReady_kickOff_opponent(Center& role, const Vector2f& ballPosition)
{
  ThresholdUtils::setThreshholdsHeigh(role);
  const float xPos = -theFieldDimensions.xPosOpponentPenaltyArea / 1.2f;
  const float yPos = 500.f;
  PositionUtils::setPosition(role, xPos, yPos);
  PositionUtils::turnTowardsBall(role, theBallSymbols);
}

float CenterProvider::goalKick_own(Center& role, bool left)
{
  regularPlay(role);
  return 0;
}

float CenterProvider::goalKick_opponent(Center& role, bool left)
{
  regularPlay(role);
  return 0;
}

float CenterProvider::pushingFreeKick_own(Center& role)
{
  regularPlay(role);
  return 0;
}

float CenterProvider::pushingFreeKick_opponent(Center& role)
{
  regularPlay(role);
  return 0;
}

float CenterProvider::cornerKick_own(Center& role, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(role);
  return 0;
}

float CenterProvider::cornerKick_opponent(Center& role, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(role);
  return 0;
}

float CenterProvider::kickIn_own(Center& role, bool left)
{
  regularPlay(role);
  return 0;
}

float CenterProvider::kickIn_opponent(Center& role, bool left)
{
  regularPlay(role);
  return 0;
}

float CenterProvider::stateReady_penaltyKick_own(Center& role)
{
  regularPlay(role);
  return 0;
}

float CenterProvider::stateReady_penaltyKick_opponent(Center& role)
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

void CenterProvider::regularPlay(Center& role)
{
  ThresholdUtils::setThreshholdsExtremeHeigh(role, theTacticSymbols.activity);

  PositionUtils::turnTowardsBall(role, theBallSymbols);

  float xPos = MathUtils::clamp_f(theBallSymbols.ballPositionField.x() - 1000.f, theFieldDimensions.xPosOwnPenaltyArea + 500.f, theFieldDimensions.xPosOpponentPenaltyArea);
  //translates the ball Y position to -1..1
  float relativeYBallPosition = MathUtils::clamp_f(theBallSymbols.ballPositionField.y() / theFieldDimensions.yPosLeftSideline, -1.0f, 1.0f);
  //max y deviation = half of field width
  float maxYDeviation = theFieldDimensions.yPosLeftSideline;
  //deviate from planned y pos based on the ball position
  float yDeviation = relativeYBallPosition * maxYDeviation;

  float yPos = MathUtils::clamp_f(500.f + yDeviation, theFieldDimensions.yPosRightSideline * 0.7f, theFieldDimensions.yPosLeftSideline * 0.7f);
  PositionUtils::setPosition(role, xPos, yPos);
}

MAKE_MODULE(CenterProvider, behaviorControl)
