/**
 * @file WhistleHandlerDortmund.cpp
 */

#include "WhistleHandlerDortmund.h"
#include "Tools/Settings.h"

MAKE_MODULE(WhistleHandlerDortmund, cognitionInfrastructure)

void WhistleHandlerDortmund::update(GameInfo& gameInfo)
{
  //printf("WhistleHandlerDortmund#update\n");
  bool whistleDetected = false;
  if (gameInfo.state == STATE_PLAYING && theRawGameInfo.state == STATE_SET)
    whistleDetected = true;
  gameInfo = theRawGameInfo;
  // no whistle detection in penalty shootout (Rules 2017)
  if (Global::getSettings().gameMode == Settings::penaltyShootout)
    return;
  if (theRawGameInfo.state != STATE_SET && theRawGameInfo.state != STATE_PLAYING)
    gameInfo.whistleCausedPlay = false;
  if (whistleDetected)
    gameInfo.state = STATE_PLAYING;
  if (gameInfo.state != lastGameState && gameInfo.state == STATE_SET)
  {
    gameInfo.whistleCausedPlay = false;
    for (unsigned int i = 0; i <= MAX_NUM_PLAYERS; i++)
      whistleTimestamps[i] = 0;
    timeOfLastSetState = theFrameInfo.time;
  }
    
  
  if (gameInfo.state != STATE_SET)
    overrideGameState = false;
  else if (!overrideGameState && gameInfo.gamePhase == GAME_PHASE_NORMAL)
  {
    gameInfo.whistleCausedPlay = checkWhistles();
    overrideGameState = gameInfo.whistleCausedPlay || checkBall();
  }
  if(overrideGameState)
  {
    if (checkForIllegalMotionPenalty())
    {
      timeOfLastSetState = theFrameInfo.time;
      overrideGameState = false;
      gameInfo.state = STATE_SET;
    }
    else
    {
      gameInfo.state = STATE_PLAYING;
    }
  }
  lastGameState = gameInfo.state;

}

bool WhistleHandlerDortmund::checkWhistles()
{
  int numberWhistleDetected = 0;
  if (theFrameInfo.getTimeSince(theWhistleDortmund.lastDetectionTime) < timeWindow)
    numberWhistleDetected++;

  int numberOfActiveTeammates = 0;
  for (auto &mate : theTeammateData.teammates)
  {
    whistleTimestamps[mate.number] = mate.whistle.lastDetectionTime;
    if (mate.whistleCausedPlay)
      return true;
    if (mate.isNDevilsPlayer && mate.status >= Teammate::ACTIVE)
      numberOfActiveTeammates++;
  }

  for (unsigned int i = 0; i <= MAX_NUM_PLAYERS; i++)
    if (theFrameInfo.getTimeSince(whistleTimestamps[i]) < timeWindow)
      numberWhistleDetected++;

  bool teamAgrees = numberWhistleDetected * 100 / (theTeammateData.numberOfActiveTeammates + 1) > percentOfTeamAgrees;
  return teamAgrees;
}

bool WhistleHandlerDortmund::checkBall()
{
  // defenders always use the ball position
  if (!useBallPosition)// && theRobotPose.translation.x() > theFieldDimensions.xPosOwnGroundline / 3)
    return false;
  Vector2f ballField = Transformation::robotToField(theRobotPose, theBallModel.estimate.position);
  return (theFrameInfo.getTimeSince(timeOfLastSetState) > 5000
    && ((theRemoteBallModel.validity > 0.7f
      && (theBallModel.validity > 0.7f || theRobotPose.translation.x() < -1500)
      && theFrameInfo.getTimeSince(theRemoteBallModel.timeWhenLastSeen) < 1000) ||
      (theBallModel.estimate.position.norm() < 1000.f && theBallModel.validity > 0.7f && theBallModel.estimate.velocity.norm() < 30.f))
    && (theRemoteBallModel.position.squaredNorm() > maxBallToMiddleDistance * maxBallToMiddleDistance ||
      (ballField.squaredNorm() > maxBallToMiddleDistance * maxBallToMiddleDistance && theBallModel.estimate.position.norm() < 1000.f)));
}

bool WhistleHandlerDortmund::checkForIllegalMotionPenalty()
{
  constexpr int minPlayerNum = Settings::lowestValidPlayerNumber;
  constexpr int maxPlayerNum = Settings::highestValidPlayerNumber;

  if (penaltyTimes.size() != static_cast<unsigned int>(maxPlayerNum))
    penaltyTimes.resize(maxPlayerNum, 0);

  for (int i = minPlayerNum; i <= maxPlayerNum; ++i)
  {
    if (theOwnTeamInfo.players[i - 1].penalty == PENALTY_SPL_ILLEGAL_MOTION_IN_SET)
    {
      if (penaltyTimes[i - 1] == 0u)
        penaltyTimes[i - 1] = theFrameInfo.time;
    }
    else
      penaltyTimes[i - 1] = 0u;
  }

  for (int i = minPlayerNum; i <= maxPlayerNum; ++i)
    if (penaltyTimes[i - 1] > timeOfLastSetState)
      return true;
  return false;
}
