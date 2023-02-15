/** 
* @file GameSymbolsProvider.cpp
*
* Implementation of class GameSymbolsProvider.
*
* @author Elena Erdmann
* @author Ingmar Schwarz
*/

#include "GameSymbolsProvider.h"
#include "stdint.h"

void GameSymbolsProvider::update(GameSymbols& gameSymbols)
{
  if (theGameInfo.state != lastGameState)
  {
    gameSymbols.lastGameState = lastGameState;
    timeWhenGameStateChanged = theFrameInfo.time;
  }
  gameSymbols.timeSinceGameState = theFrameInfo.getTimeSince(timeWhenGameStateChanged);
  ballPositionField = theBallSymbols.ballPositionField;
  bool ownKickOff = (theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam);
  gameSymbols.ownKickOff = ownKickOff;

  if (theGameInfo.state == STATE_PLAYING && lastGameState != STATE_PLAYING)
  {
    gameSymbols.ballKickedOutOfCenterCircle = false;
    gameSymbols.avoidCenterCircle = true;
    ballStartCounter = 0.f;
    ballAtStart = Vector2f(0, 0);
    timeWhenChangedToPlayingState = theFrameInfo.time;
    if (theFrameInfo.getTimeSince(theGameInfo.timeLastPackageReceived) < 5000 && (lastSecsRemaining - 10 > theGameInfo.secsRemaining))
      timeWhenChangedToPlayingState = theFrameInfo.time - (600 - theGameInfo.secsRemaining) * 1000;
    else
      gameSymbols.kickoffInProgress = true;
    counter = 0;
  }

  if (theRawGameInfo.state == STATE_PLAYING)
    gameSymbols.avoidCenterCircle = false;

  gameSymbols.timeSincePlayingState = theFrameInfo.getTimeSince(timeWhenChangedToPlayingState);

  if (gameSymbols.timeSincePlayingState < 500)
  {
    ballAtStart += ballPositionField;
    ballStartCounter++;
  }
  ballAtStart /= ballStartCounter;

  if (gameSymbols.kickoffInProgress)
  {
    const Teammate* teammate = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::kickOffFinished);
    if (teammate && theFrameInfo.getTimeSince(teammate->timeWhenSent) < 1000)
      gameSymbols.kickoffInProgress = false;
    if (gameSymbols.timeSincePlayingState > 2000 && theBallSymbols.ballWasSeen && theBallSymbols.ballWasSeen && ballPositionField.norm() > 400.f)
    {
      gameSymbols.kickoffInProgress = false;
    }
  }

  if (!gameSymbols.kickoffInProgress && gameSymbols.timeSincePlayingState > 2000 && theBallSymbols.ballWasSeen && theBallSymbols.ballWasSeen
      && ballPositionField.norm() > theFieldDimensions.centerCircleRadius + 200.f)
  {
    gameSymbols.ballKickedOutOfCenterCircle = true;
  }

  if (gameSymbols.avoidCenterCircle
      && (!gameSymbols.kickoffInProgress || gameSymbols.timeSincePlayingState > 10000 || ownKickOff
          || (theBallSymbols.ballWasSeen && ballPositionField.norm() > 400 && gameSymbols.timeSincePlayingState > 1000)))
  {
    gameSymbols.avoidCenterCircle = false;
  }

  ownTeamScoredGoal = false;
  unsigned char ownScore = theOwnTeamInfo.score;
  if (ownScore > lastOwnScore)
  {
    timeSinceScoreChange = theFrameInfo.time;
  }
  if (theFrameInfo.getTimeSince(timeSinceScoreChange) < 5000)
  {
    if (lastOwnScore < ownScore)
      lastOwnScore = ownScore;
    ownTeamScoredGoal = true;
  }
  if (lastOwnScore > ownScore)
    lastOwnScore = ownScore;


  lastGameState = theGameInfo.state;
  lastSecsRemaining = theGameInfo.secsRemaining;

  if (theRobotInfo.penalty != PENALTY_NONE)
  {
    lastPenalizedTime = theFrameInfo.time;
  }

  gameSymbols.timeSinceLastPenalty = theFrameInfo.getTimeSince(lastPenalizedTime);

  gameSymbols.allowedInPenaltyArea = calcAllowedInOwnPenaltyArea();

  // update set play state and handle changes
  bool setPlayChanged = lastSetPlay != theGameInfo.setPlay; // set play changed since last frame
  if (setPlayChanged)
    gameSymbols.lastSetPlay = gameSymbols.currentSetPlay;
  gameSymbols.currentSetPlay = theGameInfo.setPlay;
  bool kickingTeamChanged = lastKickingTeam != theGameInfo.kickingTeam; // the kicking team changed since last frame
  lastKickingTeam = theGameInfo.kickingTeam; // save kicking team to check change next frame
  if (setPlayChanged || (gameSymbols.currentSetPlay != SET_PLAY_NONE && kickingTeamChanged))
  {
    timeWhenSetPlayStarted = theFrameInfo.time;
    // check for set plays that get triggered when the ball left the field
    bool ballLeftField = (gameSymbols.currentSetPlay == SET_PLAY_CORNER_KICK || gameSymbols.currentSetPlay == SET_PLAY_GOAL_KICK || gameSymbols.currentSetPlay == SET_PLAY_KICK_IN);
    if (ballLeftField)
    {
      timeOfLastBallout = theFrameInfo.time;
      gameSymbols.lastBallOutTeam = theGameInfo.kickingTeam;
    }
  }
  gameSymbols.timeSinceSetPlayStarted = (int)(theFrameInfo.getTimeSince(timeWhenSetPlayStarted) / 1000);
  if (setPlayChanged && gameSymbols.currentSetPlay == SET_PLAY_NONE)
    timeWhenSetPlayFinished = theFrameInfo.time;
  gameSymbols.timeSinceSetPlayFinished = theFrameInfo.getTimeSince(timeWhenSetPlayFinished) / 1000;
  lastSetPlay = theGameInfo.setPlay;
  // only update ball out time after the first ball out
  if (timeOfLastBallout != -1)
  {
    gameSymbols.timeSinceLastBallOut = (int)(theFrameInfo.getTimeSince(timeOfLastBallout) / 1000);
  }
}

/**
* \brief Calculates whether or not, this robot is allowed in the own teams penalty area.
*
* The robot is allowed in its penalty area, if all of the following conditions are true:
*   - It is either the keeper, replacementKeeper, defenderSingle or ballchaser.
*   - Less than three other robots are currently positioned in the penalty area or the robot is 
*			already in the penalty area.
*
* \return true, if the robot is allowed in the penalty area, false otherwise.
*/
bool GameSymbolsProvider::calcAllowedInOwnPenaltyArea()
{
  std::unordered_set<BehaviorData::RoleAssignment> privilegedRoles = {BehaviorData::keeper, BehaviorData::replacementKeeper, BehaviorData::defenderSingle, BehaviorData::ballchaser};
  Vector2f penAreaBottomLeft(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightPenaltyArea);
  Vector2f penAreaTopRight(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  int numOfTeammatesInPenaltyArea = 0;
  for (auto const& teammate : theTeammateData.teammates)
  {
    if (Geometry::isPointInsideRectangle(penAreaBottomLeft, penAreaTopRight, teammate.pose.translation))
      numOfTeammatesInPenaltyArea++;
  }
  bool hasPrivilegedRole = privilegedRoles.find(theRoleSymbols.role) != privilegedRoles.end();
  bool penAreaIsOccupied = numOfTeammatesInPenaltyArea >= 3;
  bool robotIsInPenArea = Geometry::isPointInsideRectangle(penAreaBottomLeft, penAreaTopRight, theRobotPoseAfterPreview.translation);
  return hasPrivilegedRole && (!penAreaIsOccupied || robotIsInPenArea);
}

MAKE_MODULE(GameSymbolsProvider, behaviorControl)
