/** 
* @file GameSymbolsProvider.cpp
*
* Implementation of class GameSymbolsProvider.
*
* @author Elena Erdmann
* @author Ingmar Schwarz
*/

#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h>
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
    if (theGameInfo.controllerConnected && (lastSecsRemaining - 10 > theGameInfo.secsRemaining))
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
    if (teammate && theFrameInfo.getTimeSince(teammate->sendTimestamp) < 1000)
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

  gameSymbols.allowedInGoalArea = calcAllowedInOwnGoalArea();

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

  decideGameSituation(gameSymbols);

  if (KickUtils::isBallKicked(theMotionInfo))
  {
    gameSymbols.lastKickTime = theFrameInfo.time;
    gameSymbols.kickedThisFrame = true;
  }
  else
  {
    gameSymbols.kickedThisFrame = false;
  }
}

/**
* \brief Calculates whether or not, this robot is allowed in the own teams goal area.
*
* The robot is allowed in its goal area, if all of the following conditions are true:
*   - It is either the keeper, replacementKeeper, defenderSingle or ballchaser.
*   - Less than three other robots are currently positioned in the goal area or the robot is 
*			already in the goal area.
*
* \return true, if the robot is allowed in the goal area, false otherwise.
*/
bool GameSymbolsProvider::calcAllowedInOwnGoalArea()
{
  std::unordered_set<BehaviorData::RoleAssignment> privilegedRoles = {
      BehaviorData::keeper, BehaviorData::replacementKeeper, BehaviorData::defenderSingle, BehaviorData::defenderLeft, BehaviorData::defenderRight};
  Vector2f goalAreaBottomLeft(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoalArea);
  Vector2f goalAreaTopRight(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea);
  int numOfTeammatesInGoalArea = 0;
  for (auto const& teammate : theTeammateData.teammates)
  {
    if (Geometry::isPointInsideRectangle(goalAreaBottomLeft, goalAreaTopRight, teammate.robotPose.translation))
      numOfTeammatesInGoalArea++;
  }
  bool hasPrivilegedRole = privilegedRoles.find(theRoleSymbols.role) != privilegedRoles.end();
  bool goalAreaIsOccupied = numOfTeammatesInGoalArea >= 3;
  bool robotIsInGoalArea = Geometry::isPointInsideRectangle(goalAreaBottomLeft, goalAreaTopRight, theRobotPoseAfterPreview.translation);
  return hasPrivilegedRole && (!goalAreaIsOccupied || robotIsInGoalArea);
}

void GameSymbolsProvider::decideGameSituation(GameSymbols& theGameSymbols)
{
  switch (theGameInfo.gamePhase)
  {
  case GAME_PHASE_TIMEOUT:
    theGameSymbols.gameSituation = GameSymbols::GameSituation::none;
    return;
  case GAME_PHASE_PENALTYSHOOT:
    switch (theGameInfo.state)
    {
    case STATE_SET:
      theGameSymbols.gameSituation = GameSymbols::GameSituation::none;
      return;
    case STATE_PLAYING:
      if (theGameSymbols.ownKickOff)
      {
        theGameSymbols.gameSituation = GameSymbols::GameSituation::penaltyKick_own_playing;
      }
      else
      {
        theGameSymbols.gameSituation = GameSymbols::GameSituation::penaltyKick_opponent_playing;
      }
      return;
    }
    return;
  case GAME_PHASE_NORMAL:
  case GAME_PHASE_OVERTIME:
    switch (theGameInfo.state)
    {
    case STATE_INITIAL:
    case STATE_STANDBY:
    {
      if (theGameSymbols.ownKickOff)
      {
        theGameSymbols.gameSituation = GameSymbols::GameSituation::kickOff_own_playing_ballNotFree;
      }
      else
      {
        theGameSymbols.gameSituation = GameSymbols::GameSituation::kickOff_opponent_playing_ballNotFree;
      }
      return;
    }
    case STATE_READY:
    {
      if (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK)
      {
        // penalty kick
        if (theGameSymbols.ownKickOff)
        {
          theGameSymbols.gameSituation = GameSymbols::GameSituation::penaltyKick_own_ready;
        }
        else
        {
          theGameSymbols.gameSituation = GameSymbols::GameSituation::penaltyKick_opponent_ready;
        }
      }
      else
      {
        // kick off
        if (theGameSymbols.ownKickOff)
        {
          theGameSymbols.gameSituation = GameSymbols::GameSituation::kickOff_own_ready;
        }
        else
        {
          theGameSymbols.gameSituation = GameSymbols::GameSituation::kickOff_opponent_ready;
        }
      }
      return;
    }
    case STATE_SET:
      if (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK)
      {
        // penalty kick
        if (theGameSymbols.ownKickOff)
        {
          theGameSymbols.gameSituation = GameSymbols::GameSituation::penaltyKick_own_set;
        }
        else
        {
          theGameSymbols.gameSituation = GameSymbols::GameSituation::penaltyKick_opponent_set;
        }
      }
      else
      {
        // kick off
        if (theGameSymbols.ownKickOff)
        {
          theGameSymbols.gameSituation = GameSymbols::GameSituation::kickOff_own_set;
        }
        else
        {
          theGameSymbols.gameSituation = GameSymbols::GameSituation::kickOff_opponent_set;
        }
      }
      return;
    case STATE_PLAYING:
    {
      if (theGameSymbols.kickoffInProgress && theGameSymbols.timeSincePlayingState < 10000 && theGameInfo.setPlay == SET_PLAY_NONE) // TODO KickOff time constant
      {
        if (theGameSymbols.ownKickOff)
        {
          theGameSymbols.gameSituation = GameSymbols::GameSituation::kickOff_own_playing_ballNotFree;
          return;
        }
        else if (theGameSymbols.avoidCenterCircle)
        {
          theGameSymbols.gameSituation = GameSymbols::GameSituation::kickOff_opponent_playing_ballNotFree;
          return;
        }
        else
        {
          OUTPUT_ERROR("Unknown");
        }
      }

      if (theGameInfo.isSetPlay())
      {
        switch (getSetPlay())
        {
        case SET_PLAY_GOAL_KICK:
          if (theGameSymbols.ownKickOff)
          {
            theGameSymbols.gameSituation = GameSymbols::GameSituation::goalKick_own;
          }
          else
          {
            theGameSymbols.gameSituation = GameSymbols::GameSituation::goalKick_opponent;
          }
          return;
        case SET_PLAY_PUSHING_FREE_KICK:
          if (theGameSymbols.ownKickOff)
          {
            theGameSymbols.gameSituation = GameSymbols::GameSituation::pushingFreeKick_own;
          }
          else
          {
            theGameSymbols.gameSituation = GameSymbols::GameSituation::pushingFreeKick_opponent;
          }
          return;
        case SET_PLAY_CORNER_KICK:
        {
          if (theGameSymbols.ownKickOff)
          {
            theGameSymbols.gameSituation = GameSymbols::GameSituation::cornerKick_own;
          }
          else
          {
            theGameSymbols.gameSituation = GameSymbols::GameSituation::cornerKick_opponent;
          }
          return;
        }
        case SET_PLAY_KICK_IN:
          if (theGameSymbols.ownKickOff)
          {
            theGameSymbols.gameSituation = GameSymbols::GameSituation::kickIn_own;
          }
          else
          {
            theGameSymbols.gameSituation = GameSymbols::GameSituation::kickIn_opponent;
          }
          return;
        case SET_PLAY_PENALTY_KICK:
          if (theGameSymbols.ownKickOff)
          {
            theGameSymbols.gameSituation = GameSymbols::GameSituation::penaltyKick_own_playing;
          }
          else
          {
            theGameSymbols.gameSituation = GameSymbols::GameSituation::penaltyKick_opponent_playing;
          }
          return;
        default:
          throw std::invalid_argument("Unknown Setplay!");
        }
      }

      theGameSymbols.gameSituation = GameSymbols::GameSituation::regularPlay;

      return;
    }
    case STATE_FINISHED:
      theGameSymbols.gameSituation = GameSymbols::GameSituation::none;
      return;
    default:
      OUTPUT_ERROR("Unknown GameSituation!");
      theGameSymbols.gameSituation = GameSymbols::GameSituation::none;
      return;
    }
  }
}

int GameSymbolsProvider::getSetPlay()
{
  return theGameInfo.setPlay != SET_PLAY_NONE ? theGameInfo.setPlay : lastSetPlay; // todo is this actually required?
}

MAKE_MODULE(GameSymbolsProvider, behaviorControl)
