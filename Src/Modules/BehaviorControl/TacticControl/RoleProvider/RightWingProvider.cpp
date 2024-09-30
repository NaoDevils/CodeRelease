#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h>
#include "RightWingProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Utils/PositionUtils.h"
#include "Utils/ThresholdUtils.h"
#include "Utils/BlockedUtils.h"

void RightWingProvider::update(RightWing& role)
{
  DECLARE_DEBUG_DRAWING("behavior:RightWingProvider", "drawingOnField");
  role.stopAtTarget = true;

  bool isStateSetOrPlaying = theGameInfo.state == STATE_SET || theGameInfo.state == STATE_PLAYING;
  Vector2f ballPosition = isStateSetOrPlaying ? theBallSymbols.ballPositionFieldPredicted : Vector2f::Zero();
  updateBallIsLeft(ballPosition);

  decide(role, theBallSymbols, theFieldDimensions, theGameInfo, theGameSymbols);
}

void RightWingProvider::stateReady_kickOff_own(RightWing& role, const Vector2f& ballPosition)
{
  ThresholdUtils::setThreshholdsHeigh(role);
  const float xPos = -theFieldDimensions.xPosOpponentPenaltyArea / 7.f;
  const float yPos = theFieldDimensions.yPosRightPenaltyArea;
  PositionUtils::setPosition(role, xPos, yPos);
  PositionUtils::turnToPosition(role, {700.f, 0.f});
}

void RightWingProvider::stateReady_kickOff_opponent(RightWing& role, const Vector2f& ballPosition)
{
  ThresholdUtils::setThreshholdsHeigh(role);
  const float xPos = -theFieldDimensions.xPosOpponentPenaltyArea / 2;
  const float yPos = theFieldDimensions.yPosRightPenaltyArea;
  PositionUtils::setPosition(role, xPos, yPos);
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
  ThresholdUtils::setThreshholdsHeigh(role);
  float xPos = ballIsLeft
      ? MathUtils::clamp_f(theBallSymbols.ballPositionField.x() + 800.f, theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.xPosOpponentPenaltyArea - 200.f)
      : MathUtils::clamp_f(theBallSymbols.ballPositionField.x() + 1800.f, theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.xPosOpponentPenaltyArea);
  float yPos = ballIsLeft ? theFieldDimensions.yPosRightSideline + 2500.f : theFieldDimensions.yPosRightSideline + 100.f;
  PositionUtils::setPosition(role, xPos, yPos);
  PositionUtils::turnTowardsBall(role, theBallSymbols);
  return 0;
}

float RightWingProvider::cornerKick_opponent(RightWing& role, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(role);
  return 0;
}

float RightWingProvider::kickIn_own(RightWing& role, bool left)
{
  ThresholdUtils::setThreshholdsHeigh(role);
  float xPos = ballIsLeft
      ? MathUtils::clamp_f(theBallSymbols.ballPositionField.x() + 800.f, theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.xPosOpponentPenaltyArea)
      : MathUtils::clamp_f(theBallSymbols.ballPositionField.x() + 1800.f, theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.xPosOpponentPenaltyArea);
  float yPos = ballIsLeft ? theFieldDimensions.yPosRightSideline + 2500.f : theFieldDimensions.yPosRightSideline + 100.f;
  PositionUtils::setPosition(role, xPos, yPos);
  PositionUtils::turnTowardsBall(role, theBallSymbols);
  return 0;
}

float RightWingProvider::kickIn_opponent(RightWing& role, bool left)
{
  regularPlay(role);
  return 0;
}

float RightWingProvider::stateReady_penaltyKick_own(RightWing& role)
{
  //Get out of opponent penalty area
  ThresholdUtils::setThreshholdsHeigh(role);
  float xPos = MathUtils::clamp_f(
      theBallSymbols.ballPositionField.x() + 500.f, theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.xPosOpponentPenaltyArea + theFieldDimensions.xPosOpponentPenaltyArea / 3);
  float yPos = theFieldDimensions.yPosRightSideline + 100.f;
  PositionUtils::setPosition(role, xPos, yPos);
  PositionUtils::turnTowardsBall(role, theBallSymbols);
  return 1.f;
}

float RightWingProvider::stateReady_penaltyKick_opponent(RightWing& role)
{
  //Get out of own penalty area
  ThresholdUtils::setThreshholdsHeigh(role);
  float xPos = MathUtils::clamp_f(
      theBallSymbols.ballPositionField.x() + 500.f, theFieldDimensions.xPosOwnPenaltyArea + theFieldDimensions.xPosOwnPenaltyArea / 3, theFieldDimensions.xPosOpponentPenaltyArea);
  float yPos = theFieldDimensions.yPosRightSideline + 100.f;
  PositionUtils::setPosition(role, xPos, yPos);
  PositionUtils::turnTowardsBall(role, theBallSymbols);
  return 1.f;
}

void RightWingProvider::regularPlay(RightWing& role)
{
  ThresholdUtils::setThreshholdsExtremeHeigh(role, theTacticSymbols.activity);
  PositionUtils::turnTowardsBall(role, theBallSymbols);

  const float yOffset = theFieldDimensions.yPosLeftSideline / 6;
  float xPos = MathUtils::clamp_f(
      theBallSymbols.ballPositionField.x() + 500.f - 2000.f * (float)theTacticSymbols.defensiveBehavior, theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.xPosOpponentPenaltyArea);
  //translates the ball Y position to -1..1
  float relativeYBallPosition = MathUtils::clamp_f(theBallSymbols.ballPositionField.y() / theFieldDimensions.yPosLeftSideline, -1.0f, 1.0f);
  //max y deviation = half of field width
  float maxYDeviation = theFieldDimensions.yPosLeftSideline;
  //deviate (mostly to the left) from planned y pos based on the ball position
  float yDeviation = MathUtils::clamp_f(relativeYBallPosition * maxYDeviation, -yOffset, maxYDeviation);

  float yPos = theFieldDimensions.yPosRightSideline + yOffset * 3 - yOffset * (float)theTacticSymbols.defensiveBehavior + yDeviation;
  LINE("behavior:RightWingProvider", xPos, yPos, theBallSymbols.ballPositionField.x(), theBallSymbols.ballPositionField.y(), 10, Drawings::dashedPen, ColorRGBA::orange);

  float activity = theTacticSymbols.activity;
  Vector2f newPosition;
  if (theRoleSymbols.role == BehaviorData::rightWing)
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
  CROSS("behavior:RightWingProvider", xPos, yPos, 100, 50, Drawings::solidPen, ColorRGBA::orange);
  CROSS("behavior:RightWingProvider", newPosition.x(), newPosition.y(), 100, 10, Drawings::solidPen, ColorRGBA::orange);
  LINE("behavior:RightWingProvider", newPosition.x(), newPosition.y(), theBallSymbols.ballPositionField.x(), theBallSymbols.ballPositionField.y(), 10, Drawings::solidPen, ColorRGBA::orange);
}

/**
 * @brief Avoids constant changes of the value
*/
void RightWingProvider::updateBallIsLeft(const Vector2f& ballPosition)
{
  if (ballIsLeft && ballPosition.y() < -250.f)
    ballIsLeft = false;
  if (!ballIsLeft && ballPosition.y() > 250.f)
    ballIsLeft = true;
}

MAKE_MODULE(RightWingProvider, behaviorControl)
