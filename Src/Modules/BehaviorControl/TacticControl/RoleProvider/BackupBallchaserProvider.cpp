#include "BackupBallchaserProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Utils/PositionUtils.h"
#include "Utils/ThresholdUtils.h"
#include "Utils/MathUtils.h"
#include "Utils/BlockedUtils.h"

void BackupBallchaserProvider::update(BackupBallchaser& role)
{
  DECLARE_DEBUG_DRAWING("behavior:BackWingProvider", "drawingOnField");
  role.stopAtTarget = true;

  bool isStateSetOrPlaying = theGameInfo.state == STATE_SET || theGameInfo.state == STATE_PLAYING;
  Vector2f ballPosition = isStateSetOrPlaying ? theBallSymbols.ballPositionFieldPredicted : Vector2f::Zero();
  updateBallIsLeft(ballPosition);

  decide(role, theBallSymbols, theFieldDimensions, theGameInfo, theGameSymbols);
}

void BackupBallchaserProvider::stateReady_kickOff_own(BackupBallchaser& role, const Vector2f& ballPosition)
{
  regularPlay(role);
}

void BackupBallchaserProvider::stateReady_kickOff_opponent(BackupBallchaser& role, const Vector2f& ballPosition)
{
  ThresholdUtils::setThreshholdsHeigh(role);
  const float xPos = -theFieldDimensions.xPosOpponentPenaltyArea / 1.2f;
  const float yPos = 500.f;
  PositionUtils::setPosition(role, xPos, yPos);
  PositionUtils::turnTowardsBall(role, theBallSymbols);
}

float BackupBallchaserProvider::goalKick_own(BackupBallchaser& role, bool left)
{
  regularPlay(role);
  return 0.f;
}

float BackupBallchaserProvider::goalKick_opponent(BackupBallchaser& role, bool left)
{
  regularPlay(role);
  return 0;
}

float BackupBallchaserProvider::pushingFreeKick_own(BackupBallchaser& role)
{
  regularPlay(role);
  return 0;
}

float BackupBallchaserProvider::pushingFreeKick_opponent(BackupBallchaser& role)
{
  regularPlay(role);
  return 0;
}

float BackupBallchaserProvider::cornerKick_own(BackupBallchaser& role, const Vector2f& cornerKickPosition, bool left)
{
  ThresholdUtils::setThreshholdsHeigh(role);
  float xPos = MathUtils::clamp_f(theBallSymbols.ballPositionField.x() - 2000.f, theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.xPosOpponentPenaltyArea / 2);
  float yPos = ballIsLeft ? theFieldDimensions.yPosLeftSideline - 1800.f : theFieldDimensions.yPosRightSideline + 1800.f;
  PositionUtils::setPosition(role, xPos, yPos);
  PositionUtils::turnTowardsBall(role, theBallSymbols);
  return 0;
}

float BackupBallchaserProvider::cornerKick_opponent(BackupBallchaser& role, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(role);
  return 0;
}

float BackupBallchaserProvider::kickIn_own(BackupBallchaser& role, bool left)
{
  ThresholdUtils::setThreshholdsHeigh(role);
  float xPos = MathUtils::clamp_f(theBallSymbols.ballPositionField.x() - 2000.f, theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.xPosOpponentPenaltyArea / 2);
  float yPos = ballIsLeft ? theFieldDimensions.yPosLeftSideline - 1800.f : theFieldDimensions.yPosRightSideline + 1800.f;
  PositionUtils::setPosition(role, xPos, yPos);
  PositionUtils::turnTowardsBall(role, theBallSymbols);
  return 0;
}

float BackupBallchaserProvider::kickIn_opponent(BackupBallchaser& role, bool left)
{
  regularPlay(role);
  return 0;
}

float BackupBallchaserProvider::stateReady_penaltyKick_own(BackupBallchaser& role)
{
  regularPlay(role);
  return 0;
}

float BackupBallchaserProvider::stateReady_penaltyKick_opponent(BackupBallchaser& role)
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

void BackupBallchaserProvider::regularPlay(BackupBallchaser& role)
{
  ThresholdUtils::setThreshholdsExtremeHeigh(role, theTacticSymbols.activity);

  //brief Get position behind ballchaser.
  //assume ballchaser always positions in front of the ball
  float xPos = MathUtils::clamp_f(theBallSymbols.ballPositionField.x() - 1000.f, theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.xPosOpponentPenaltyArea);
  //get position between ballchaser and goal
  float yDeviation = ballIsLeft ? theBallSymbols.ballPositionField.y() + theFieldDimensions.yPosRightGoal : theBallSymbols.ballPositionField.y() + theFieldDimensions.yPosLeftGoal;
  float yPos = MathUtils::clamp_f(yDeviation, theFieldDimensions.yPosRightSideline * 0.7f, theFieldDimensions.yPosLeftSideline * 0.7f);

  LINE("behavior:BackupBallchaserProvider", xPos, yPos, theBallSymbols.ballPositionField.x(), theBallSymbols.ballPositionField.y(), 10, Drawings::dashedPen, ColorRGBA::yellow);

  ThresholdUtils::setThreshholdsExtremeHeigh(role, theTacticSymbols.activity);

  PositionUtils::setPosition(role, xPos, yPos);
  PositionUtils::turnTowardsBall(role, theBallSymbols);
}

/**
 * @brief Avoids constant changes of the value
*/
void BackupBallchaserProvider::updateBallIsLeft(const Vector2f& ballPosition)
{
  if (ballIsLeft && ballPosition.y() < -250.f)
    ballIsLeft = false;
  if (!ballIsLeft && ballPosition.y() > 250.f)
    ballIsLeft = true;
}

MAKE_MODULE(BackupBallchaserProvider, behaviorControl)
