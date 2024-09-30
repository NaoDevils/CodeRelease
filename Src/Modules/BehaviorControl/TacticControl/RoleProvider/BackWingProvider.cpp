#include "BackWingProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Utils/PositionUtils.h"
#include "Utils/ThresholdUtils.h"
#include "Utils/MathUtils.h"
#include "Utils/BlockedUtils.h"

void BackWingProvider::update(BackWing& role)
{
  DECLARE_DEBUG_DRAWING("behavior:BackWingProvider", "drawingOnField");
  role.stopAtTarget = true;

  bool isStateSetOrPlaying = theGameInfo.state == STATE_SET || theGameInfo.state == STATE_PLAYING;
  Vector2f ballPosition = isStateSetOrPlaying ? theBallSymbols.ballPositionFieldPredicted : Vector2f::Zero();
  updateBallIsLeft(ballPosition);

  decide(role, theBallSymbols, theFieldDimensions, theGameInfo, theGameSymbols);
}

void BackWingProvider::stateReady_kickOff_own(BackWing& role, const Vector2f& ballPosition)
{
  regularPlay(role);
}

void BackWingProvider::stateReady_kickOff_opponent(BackWing& role, const Vector2f& ballPosition)
{
  ThresholdUtils::setThreshholdsHeigh(role);
  const float xPos = -theFieldDimensions.xPosOpponentPenaltyArea / 1.2f;
  const float yPos = 500.f;
  PositionUtils::setPosition(role, xPos, yPos);
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
  ThresholdUtils::setThreshholdsHeigh(role);
  float xPos = MathUtils::clamp_f(theBallSymbols.ballPositionField.x() - 2000.f, theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.xPosOpponentPenaltyArea / 2);
  float yPos = ballIsLeft ? theFieldDimensions.yPosLeftSideline - 1800.f : theFieldDimensions.yPosRightSideline + 1800.f;
  PositionUtils::setPosition(role, xPos, yPos);
  PositionUtils::turnTowardsBall(role, theBallSymbols);
  return 0;
}

float BackWingProvider::cornerKick_opponent(BackWing& role, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(role);
  return 0;
}

float BackWingProvider::kickIn_own(BackWing& role, bool left)
{
  ThresholdUtils::setThreshholdsHeigh(role);
  float xPos = MathUtils::clamp_f(theBallSymbols.ballPositionField.x() - 2000.f, theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.xPosOpponentPenaltyArea / 2);
  float yPos = ballIsLeft ? theFieldDimensions.yPosLeftSideline - 1800.f : theFieldDimensions.yPosRightSideline + 1800.f;
  PositionUtils::setPosition(role, xPos, yPos);
  PositionUtils::turnTowardsBall(role, theBallSymbols);
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

  float xPos = MathUtils::clamp_f(theBallSymbols.ballPositionField.x() - 2500.f, theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.xPosOpponentPenaltyArea / 10);
  //translates the ball Y position to -1..1
  float relativeYBallPosition = MathUtils::clamp_f(theBallSymbols.ballPositionField.y() / theFieldDimensions.yPosLeftSideline, -1.0f, 1.0f);
  //max y deviation = half of field width
  float maxYDeviation = theFieldDimensions.yPosLeftSideline;
  //deviate from planned y pos based on the ball position
  float yDeviation = (theBallSymbols.ballOnLeftSide ? -500.f : 500.f) + relativeYBallPosition * maxYDeviation;
  float yPos = MathUtils::clamp_f(yDeviation, theFieldDimensions.yPosRightSideline * 0.7f, theFieldDimensions.yPosLeftSideline * 0.7f);
  LINE("behavior:BackWingProvider", xPos, yPos, theBallSymbols.ballPositionField.x(), theBallSymbols.ballPositionField.y(), 10, Drawings::dashedPen, ColorRGBA::yellow);

  float activity = theTacticSymbols.activity;
  Vector2f newPosition;
  if (theRoleSymbols.role == BehaviorData::backWing)
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

  CROSS("behavior:BackWingProvider", xPos, yPos, 100, 50, Drawings::solidPen, ColorRGBA::yellow);
  CROSS("behavior:BackWingProvider", newPosition.x(), newPosition.y(), 100, 10, Drawings::solidPen, ColorRGBA::yellow);
  LINE("behavior:BackWingProvider", newPosition.x(), newPosition.y(), theBallSymbols.ballPositionField.x(), theBallSymbols.ballPositionField.y(), 10, Drawings::solidPen, ColorRGBA::yellow);
}

/**
 * @brief Avoids constant changes of the value
*/
void BackWingProvider::updateBallIsLeft(const Vector2f& ballPosition)
{
  if (ballIsLeft && ballPosition.y() < -250.f)
    ballIsLeft = false;
  if (!ballIsLeft && ballPosition.y() > 250.f)
    ballIsLeft = true;
}

MAKE_MODULE(BackWingProvider, behaviorControl)
