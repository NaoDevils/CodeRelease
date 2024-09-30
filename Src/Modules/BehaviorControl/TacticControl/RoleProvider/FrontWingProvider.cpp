#include "FrontWingProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Utils/PositionUtils.h"
#include "Utils/ThresholdUtils.h"
#include "Utils/MathUtils.h"
#include "Utils/BlockedUtils.h"

void FrontWingProvider::update(FrontWing& role)
{
  DECLARE_DEBUG_DRAWING("behavior:FrontWingProvider", "drawingOnField");
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

  float xPos = MathUtils::clamp_f(theBallSymbols.ballPositionField.x() + 2000.f, -theFieldDimensions.xPosOpponentPenaltyArea / 2, theFieldDimensions.xPosOpponentPenaltyArea);
  //translates the ball Y position to -1..1
  float relativeYBallPosition = MathUtils::clamp_f(theBallSymbols.ballPositionField.y() / theFieldDimensions.yPosLeftSideline, -1.0f, 1.0f);
  //max y deviation = half of field width
  float maxYDeviation = theFieldDimensions.yPosLeftSideline;
  //deviate from planned y pos based on the ball position
  float yDeviation = relativeYBallPosition * maxYDeviation;

  float yPos = MathUtils::clamp_f(-500.f + yDeviation, theFieldDimensions.yPosRightSideline * 0.7f, theFieldDimensions.yPosLeftSideline * 0.7f);
  LINE("behavior:FrontWingProvider", xPos, yPos, theBallSymbols.ballPositionField.x(), theBallSymbols.ballPositionField.y(), 10, Drawings::dashedPen, ColorRGBA::red);


  float activity = theTacticSymbols.activity;
  Vector2f newPosition;
  if (theRoleSymbols.role == BehaviorData::frontWing)
  {
    role.framesSinceLastSideStep++;
    newPosition = BlockedUtils::moveIfBlocked(xPos, yPos, role.framesSinceLastSideStep, role.optPosition.translation, role.blockedByRobot, activity, theRobotMap, theBallSymbols);
  }
  else
  {
    newPosition = Vector2f(xPos, yPos);
  }

  ThresholdUtils::setThreshholdsExtremeHeigh(role, activity);
  PositionUtils::setPosition(role, newPosition.x(), newPosition.y());
  CROSS("behavior:FrontWingProvider", xPos, yPos, 100, 50, Drawings::solidPen, ColorRGBA::red);
  CROSS("behavior:FrontWingProvider", newPosition.x(), newPosition.y(), 100, 10, Drawings::solidPen, ColorRGBA::red);
  LINE("behavior:FrontWingProvider", newPosition.x(), newPosition.y(), theBallSymbols.ballPositionField.x(), theBallSymbols.ballPositionField.y(), 10, Drawings::solidPen, ColorRGBA::red);
}

MAKE_MODULE(FrontWingProvider, behaviorControl)
