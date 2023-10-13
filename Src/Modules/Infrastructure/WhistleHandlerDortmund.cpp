/**
 * @file WhistleHandlerDortmund.cpp
 */

#include "WhistleHandlerDortmund.h"
#include "Representations/Infrastructure/TeamCommEvents.h"
#include "Tools/Settings.h"

MAKE_MODULE(WhistleHandlerDortmund, cognitionInfrastructure)

void WhistleHandlerDortmund::update(GameInfo& gameInfo)
{
  //printf("WhistleHandlerDortmund#update\n");
  // check if game state was changed due to whistle -> keep that change
  bool setToPlay = gameInfo.state == STATE_PLAYING && theRawGameInfo.state == STATE_SET;
  bool playToReady = gameInfo.state == STATE_READY && theRawGameInfo.state == STATE_PLAYING;

  // check if gameInfo does not agree with rawGameInfo after GameController should have sent the update
  if (playToReady && theFrameInfo.getTimeSince(timeStampPlayToReady) > playToReadyTimeout)
    playToReady = false;
  if (setToPlay && theFrameInfo.getTimeSince(timeStampSetToPlay) > setToPlayTimeout)
    setToPlay = false;

  // update gameInfo with rawgameinfo
  gameInfo = theRawGameInfo;
  // remember time of state change to set for illegal motion detection
  if (gameInfo.state != lastGameState && gameInfo.state == STATE_SET)
  {
    for (unsigned int i = 0; i <= MAX_NUM_PLAYERS; i++)
      whistleTimestamps[i] = 0;
    timeOfLastSetState = theFrameInfo.time;
  }

  // if whistle caused a game state change, game info has to be updated accordingly
  if (setToPlay)
  {
    // illegal motion penalty can only occur in set state -> whistle detection went wrong
    // TODO: this should also trigger something in the whistleHandler!
    if (checkForIllegalMotionPenalty())
    {
      timeOfLastSetState = theFrameInfo.time;
      gameInfo.state = STATE_SET;
    }
    else
    {
      gameInfo.state = STATE_PLAYING;
    }
  }
  else if (playToReady)
    gameInfo.state = STATE_READY;
  else
  { // no game state was recently changed -> check whistle and other hints for a referee game state change
    const bool whistleDetected = theWhistleDortmund.detectionState == WhistleDortmund::DetectionState::isDetected;
    if (theRawGameInfo.state == STATE_SET && (whistleDetected || checkBall()))
    {
      gameInfo.state = STATE_PLAYING;
      timeStampSetToPlay = theFrameInfo.time;
    }

    const int localTimeDiffGoalToWhistle = static_cast<int>(theBallModel.timeWhenBallInGoalBox) - static_cast<int>(theWhistleDortmund.lastDetectionTime);
    const int remoteTimeDiffGoalToWhistle = static_cast<int>(theRemoteBallModel.timeWhenBallInGoalBox) - static_cast<int>(theWhistleDortmund.lastDetectionTime);
    const Teammate* goalDetectedMate = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::goalDetected);
    const bool inSetPlay = theRawGameInfo.setPlay != SET_PLAY_NONE;
    const bool localGoalDetected = theWhistleDortmund.lastDetectionTime > 0 && theFrameInfo.getTimeSince(theWhistleDortmund.lastDetectionTime) < 3000
        && ((std::abs(localTimeDiffGoalToWhistle) < maxTimediffWhistleToGoal && theBallModel.validity > 0.7)
            || (std::abs(remoteTimeDiffGoalToWhistle) < maxTimediffWhistleToGoal && theRemoteBallModel.validity > 0.7));
    const bool remoteGoalDetected = goalDetectedMate && theWhistleDortmund.lastDetectionTime > 0 && goalDetectedMate->whistle.lastDetectionTime > 1
        && theFrameInfo.getTimeSince(theWhistleDortmund.lastDetectionTime) < timeWindow && theFrameInfo.getTimeSince(goalDetectedMate->whistle.lastDetectionTime) < timeWindow;
    if (theRawGameInfo.state == STATE_PLAYING && !inSetPlay && (localGoalDetected || remoteGoalDetected))
    {
      gameInfo.state = STATE_READY;
      timeStampPlayToReady = theFrameInfo.time;
    }
  }
  lastGameState = gameInfo.state;
}

bool WhistleHandlerDortmund::checkBall()
{
  // defenders always use the ball position
  if (!useBallPosition) // && theRobotPose.translation.x() > theFieldDimensions.xPosOwnGroundline / 3)
    return false;
  Vector2f ballField = Transformation::robotToField(theRobotPose, theBallModel.estimate.position);
  return (theFrameInfo.getTimeSince(timeOfLastSetState) > 5000
      && ((theRemoteBallModel.validity > 0.7f && (theBallModel.validity > 0.7f || theRobotPose.translation.x() < -1500) && theFrameInfo.getTimeSince(theRemoteBallModel.timeWhenLastSeen) < 1000)
          || (theBallModel.estimate.position.norm() < 1000.f && theBallModel.validity > 0.7f && theBallModel.estimate.velocity.norm() < 30.f))
      && (theRemoteBallModel.position.squaredNorm() > maxBallToMiddleDistance * maxBallToMiddleDistance
          || (ballField.squaredNorm() > maxBallToMiddleDistance * maxBallToMiddleDistance && theBallModel.estimate.position.norm() < 1000.f)));
}

bool WhistleHandlerDortmund::checkForGoal()
{
  if (!useBallForPlayingReadyTransition)
    return true;
  Vector2f ballField = Transformation::robotToField(theRobotPose, theBallModel.estimate.position);
  Vector2f ballFieldRemote = theRemoteBallModel.position;
  bool localBallInGoalArea = theBallModel.validity > 0.7f && std::abs(ballField.x()) > theFieldDimensions.xPosOpponentGoalArea && std::abs(ballField.y()) < theFieldDimensions.yPosLeftGoalArea;
  bool remoteBallInGoalArea = theRemoteBallModel.validity > 0.7f && std::abs(ballFieldRemote.x()) > theFieldDimensions.xPosOpponentGoalArea
      && std::abs(ballFieldRemote.y()) < theFieldDimensions.yPosLeftGoalArea;
  return localBallInGoalArea || remoteBallInGoalArea;
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
