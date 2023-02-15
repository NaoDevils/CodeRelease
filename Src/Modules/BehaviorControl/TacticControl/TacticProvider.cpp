#include "TacticProvider.h"
#include <algorithm>

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
  return;
}

/**
* \brief Determines how many field player (me + team mates) are active.
*
* The keeper is not included in the total. The result will be stored in the TacticSymbols'
* numberOfActiveFieldPlayers attribute instead of being returned directly.
*/
void TacticProvider::calcNumberOfActiveFieldPlayers(TacticSymbols& tacticSymbols)
{
  tacticSymbols.numberOfActiveFieldPlayers = (theRobotInfo.number == 1 || theRobotInfo.penalty != PENALTY_NONE) ? 0 : 1; // myself
  for (auto& mate : theTeammateData.teammates)
  {
    if (mate.status >= Teammate::ACTIVE && mate.number != 1)
      tacticSymbols.numberOfActiveFieldPlayers++;
  }
}


/**
* \brief Decides between defensive and offensive behavior based on the direction in which the ball has moved last.
*
* \return True if the defensive behavior should be used, false otherwise.
*/
bool TacticProvider::decideDefensiveBehavior()
{
  int scoreDiff = theOwnTeamInfo.score - theOpponentTeamInfo.score;

  // always play defensively when leading
  if (scoreDiff > 0)
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

void TacticProvider::getBallDirection()
{
  // compute previous and current location of the ball
  TacticProvider::BallSide previousSide = TacticProvider::currentSide;
  getBallSide();
  TacticProvider::BallSide newSide = TacticProvider::currentSide;

  // compare sides to detect movement
  if ((previousSide == TacticProvider::BallSide::back && newSide == TacticProvider::BallSide::center)
      || (previousSide == TacticProvider::BallSide::center && newSide == TacticProvider::BallSide::front))
  {
    // offensive direction
    TacticProvider::currentDirection = TacticProvider::BallDirection::towardsEnemySide;
    TacticProvider::directionChanged = true;
  }
  else if ((previousSide == TacticProvider::BallSide::front && newSide == TacticProvider::BallSide::center)
      || (previousSide == TacticProvider::BallSide::center && newSide == TacticProvider::BallSide::back))
  {
    // defensive direction
    TacticProvider::currentDirection = TacticProvider::BallDirection::towardsOwnSide;
    TacticProvider::directionChanged = true;
  }
  else
  {
    // no change of direction
    TacticProvider::directionChanged = false;
  }
}

void TacticProvider::getBallSide()
{
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

MAKE_MODULE(TacticProvider, behaviorControl)
