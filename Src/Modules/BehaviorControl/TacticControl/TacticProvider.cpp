#include "TacticProvider.h"
#include <algorithm>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/RoleProvider.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/FieldUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/TacticUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/BallUtils.h>

void TacticProvider::update(TacticSymbols& tacticSymbols)
{
  if (theGameInfo.state == STATE_READY) // Decide kickoff direction
  {
    tacticSymbols.kickoffToTheLeft = decideKickoffDirection(tacticSymbols);
  }
  // Sets numberOfActiveFieldPlayers of PositioningSymbols.
  calcNumberOfActiveFieldPlayers(tacticSymbols);
  // Decides between defensive/offensive behavior based on the direction in which the ball has moved last.
  tacticSymbols.defensiveBehavior = decideDefensiveBehavior();
  tacticSymbols.activity = decideActivity();
  decideFightForBall(tacticSymbols);
  decideDefensiveCone(tacticSymbols);
  tacticSymbols.keepRoleAssignment = theGameInfo.state == STATE_READY && theGameSymbols.timeSinceGameState > static_cast<int>(timeTillKeepRoleAssignmentInReady);
}

/**
* \brief Determines how many field player (me + team mates) are active.
*
* The keeper is not included in the total. The result will be stored in the TacticSymbols'
* numberOfActiveFieldPlayers attribute instead of being returned directly.
*/
void TacticProvider::calcNumberOfActiveFieldPlayers(TacticSymbols& tacticSymbols)
{
  if (theTeammateData.wlanOK)
  {
    tacticSymbols.numberOfActiveFieldPlayers = (theRobotInfo.number == 1 || theRobotInfo.penalty != PENALTY_NONE) ? 0 : 1; // myself
    for (auto& mate : theTeammateData.teammates)
    {
      if (mate.status >= TeammateReceived::Status::ACTIVE && mate.playerNumber != 1)
        tacticSymbols.numberOfActiveFieldPlayers++;
    }
  }
  else if (theGameInfo.controllerConnected)
  {
    const auto isActive = [](const RoboCup::RobotInfo& info)
    {
      return info.penalty == PENALTY_NONE;
    };

    // Skip keeper (player == 0)
    tacticSymbols.numberOfActiveFieldPlayers = static_cast<unsigned>(std::count_if(std::begin(theOwnTeamInfo.players) + 1, std::begin(theOwnTeamInfo.players) + MAX_NUM_PLAYERS, isActive));
  }
  else
  {
    tacticSymbols.numberOfActiveFieldPlayers = MAX_NUM_PLAYERS - 1;
  }
}


/**
* \brief Decides between defensive and offensive behavior based on the direction in which the ball has moved last.
*
* \return True if the defensive behavior should be used, false otherwise.
*/
bool TacticProvider::decideDefensiveBehavior()
{
  const bool setPlayOwn = (theGameSymbols.ownKickOff && theGameSymbols.currentSetPlay != SET_PLAY_NONE);
  const bool setPlayOpponent = (!theGameSymbols.ownKickOff && theGameSymbols.currentSetPlay != SET_PLAY_NONE);
  if (setPlayOwn)
  {
    defensiveBehavior = false;
  }
  else if (setPlayOpponent)
  {
    defensiveBehavior = true;
  }
  else
  {
    getBallDirection();
    if (defensiveBehavior && currentDirection == TacticProvider::BallDirection::towardsEnemySide)
    {
      defensiveBehavior = false;
    }
    else if (!defensiveBehavior && currentDirection == TacticProvider::BallDirection::towardsOwnSide)
    {
      defensiveBehavior = true;
    }
  }
  return defensiveBehavior;
}

float TacticProvider::decideActivity()
{
  // Activity is high after kickOff and setPlays
  const bool kickOffOngoing = theGameSymbols.kickoffInProgress && theGameSymbols.timeSincePlayingState < 10000; // TODO KickOff time constant
  const bool setPlayOngoing = theGameInfo.setPlay != SET_PLAY_NONE;
  if (kickOffOngoing || setPlayOngoing)
  {
    activity = 1;
    return activity;
  }

  // Activity is high if ball direction changed
  if (lastDirection != currentDirection)
  {
    activity = 1.f;
    return activity;
  }

  // Activity is high if ball side changed
  if (lastSide != currentSide)
  {
    activity = 1.f;
    return activity;
  }

  // Else activity gets decreased
  const float ACTIVITY_LOSS = 0.001f;
  activity = std::max(activity - ACTIVITY_LOSS, 0.f);
  return activity;
}

void TacticProvider::getBallDirection()
{
  lastDirection = currentDirection;

  // compute previous and current location of the ball
  TacticProvider::BallSide previousSide = TacticProvider::currentSide;
  getBallSide();
  TacticProvider::BallSide newSide = TacticProvider::currentSide;

  // compare sides to detect movement
  if ((previousSide == TacticProvider::BallSide::back && newSide == TacticProvider::BallSide::center)
      || (previousSide == TacticProvider::BallSide::center && newSide == TacticProvider::BallSide::front))
  {
    // offensive direction
    currentDirection = TacticProvider::BallDirection::towardsEnemySide;
  }
  else if ((previousSide == TacticProvider::BallSide::front && newSide == TacticProvider::BallSide::center)
      || (previousSide == TacticProvider::BallSide::center && newSide == TacticProvider::BallSide::back))
  {
    // defensive direction
    currentDirection = TacticProvider::BallDirection::towardsOwnSide;
  }
  else
  {
    // no change of direction
  }
}

void TacticProvider::getBallSide()
{
  lastSide = currentSide;

  Vector2f ballPosition = theBallSymbols.ballPositionField;

  if (ballPosition.x() < 0.24 * theFieldDimensions.xPosOwnGroundline)
  {
    currentSide = TacticProvider::BallSide::back;
  }
  else if (ballPosition.x() > 0.21 * theFieldDimensions.xPosOwnGroundline && ballPosition.x() < 0.21 * theFieldDimensions.xPosOpponentGroundline)
  {
    currentSide = TacticProvider::BallSide::center;
  }
  else if (ballPosition.x() > 0.24 * theFieldDimensions.xPosOpponentGroundline)
  {
    currentSide = TacticProvider::BallSide::front;
  }
}

/**
* \brief Decides kickoff direction depending on to which sides kickoff was most successful.
*/
bool TacticProvider::decideKickoffDirection(TacticSymbols& tacticSymbols)
{

  // If dynamic side switching is disabled for kickoff, use kickOffToTheLeftSide parameter.
  if (!theBehaviorConfiguration.behaviorParameters.useDynamicKickoffSideSwitching)
    return theBehaviorConfiguration.behaviorParameters.kickOffToTheLeftSide;
  if (theOwnTeamInfo.score == 0 && theOpponentTeamInfo.score == 0) // Initial kickoff
    return theBehaviorConfiguration.behaviorParameters.kickOffToTheLeftSide;
  if (lastKickoffWasOwn)
  {
    if (theOwnTeamInfo.score > lastOwnScore) // Kickoff was successful
    {
      if (tacticSymbols.kickoffToTheLeft)
        tacticSymbols.numberOfLeftOwnKickOffSuccess++;
      else
        tacticSymbols.numberOfRightOwnKickOffSuccess++;
    }
    else if (theOpponentTeamInfo.score > lastOpponentScore) // Kickoff failed
    {
      if (tacticSymbols.kickoffToTheLeft)
        tacticSymbols.numberOfLeftOwnKickOffSuccess--;
      else
        tacticSymbols.numberOfRightOwnKickOffSuccess--;
    }
  }
  lastOwnScore = theOwnTeamInfo.score;
  lastOpponentScore = theOpponentTeamInfo.score;
  lastKickoffWasOwn = theGameSymbols.ownKickOff;
  return (tacticSymbols.numberOfLeftOwnKickOffSuccess >= tacticSymbols.numberOfRightOwnKickOffSuccess);
}

void TacticProvider::decideFightForBall(TacticSymbols& tacticSymbols)
{
  const int CLOSE_DISTANCE = 500;
  const bool hysteresis = tacticSymbols.closeToBallRobotNumber > 0;
  const float hysteresisMultiplier = hysteresis ? 1.1f : 1.f;
  const int adjustedCloseDistance = (int)(hysteresisMultiplier * CLOSE_DISTANCE);

  const Vector2f& ballPosition = theBallSymbols.ballPositionField;

  tacticSymbols.closeToBallRobotNumber = 0;
  tacticSymbols.closeToBallOpponentRobotNumber = 0;
  int minDistance = adjustedCloseDistance;

  for (const auto& robot : theRobotMap.robots)
  {
    const int distance = (int)Geometry::distance(robot.pose.translation, ballPosition);
    const bool opponentRobot = robot.robotType != RobotEstimate::RobotType::teammateRobot;

    if (distance <= minDistance)
    {
      minDistance = distance;

      tacticSymbols.closeToBallRobotNumber++;
      tacticSymbols.closeToBallRobot = robot.pose;
      if (opponentRobot)
      {
        tacticSymbols.closeToBallOpponentRobotNumber++;
        tacticSymbols.closeToBallOpponentRobot = robot.pose;
      }
    }
    else if (distance <= adjustedCloseDistance)
    {
      tacticSymbols.closeToBallRobotNumber++;
      if (opponentRobot)
      {
        tacticSymbols.closeToBallOpponentRobotNumber++;
      }
    }
  }
}

void TacticProvider::decideDefensiveCone(TacticSymbols& theTacticSymbols)
{
  const Vector2f ballPosition = BallUtils::getBallPosition(theBallSymbols, theFrameInfo, theRobotPoseAfterPreview);
  theTacticSymbols.defensiveCone = TacticUtils::getDefenseCone(180_deg, ballPosition, theFieldDimensions);
}

MAKE_MODULE(TacticProvider, behaviorControl)
