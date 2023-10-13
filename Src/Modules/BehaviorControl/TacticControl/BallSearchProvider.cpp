/**
* @file BallSearchProvider.cpp
*
* Implementation of class BallSearchProvider.
*
*/

#include "BallSearchProvider.h"

void BallSearchProvider::update(BallSearch& ballsearch)
{
  // General idea:
  // if ball was lost for whole team, start search with roles near ball position.
  // This does not use specific role positions but general areas (e.g. defenders lower own half, receiver offensive etc)

  // clear up list and reset members
  ballsearch.ballSearchPositions.clear();
  ballsearch.rolesInBallSearch.clear();
  defenseInBallSearch = 0;
  centerInBallSearch = 0;
  offenseInBallSearch = 0;

  // get ball position from member who saw the ball last
  int timeSinceBallLastSeen = theBallSymbols.timeSinceLastSeen;
  lastBallPositionField = theBallSymbols.ballPositionField;
  if (timeSinceBallLastSeen == static_cast<int>(theFrameInfo.time)) //never seen, assume center position
    lastBallPositionField = Vector2f::Zero();

  for (auto& mate : theTeammateData.teammates)
    if (theFrameInfo.getTimeSince(mate.ballModel.timeWhenLastSeen) < timeSinceBallLastSeen)
    {
      timeSinceBallLastSeen = theFrameInfo.getTimeSince(mate.ballModel.timeWhenLastSeen);
      lastBallPositionField = mate.behaviorData.ballPositionField.cast<float>();
    }

  // if game state not playing do not search for ball!
  if (theGameInfo.state != STATE_PLAYING)
    return;
  // if ball was not seen for some time, start ball search with nearest positions
  if (theBallSymbols.ballLostForTeam)
  {
    if (timeStampBallLostForTeam == 0)
      timeStampBallLostForTeam = theFrameInfo.time;
    int timeSinceBallSearchStarted = theFrameInfo.getTimeSince(timeStampBallLostForTeam);

    if (lastBallPositionField.x() < theFieldDimensions.xPosOwnGroundline + defenseSearchStart * 2 * theFieldDimensions.xPosOpponentGroundline)
    {
      for (auto& role : theRoleSelection.selectedRoles)
        if (role == BehaviorData::defenderLeft || role == BehaviorData::defenderRight || role == BehaviorData::defenderSingle)
        {
          ballsearch.rolesInBallSearch.push_back(BehaviorData::RoleAssignment(role));
          defenseInBallSearch++;
        }
    }
    if (timeSinceBallSearchStarted > timeUntilWholeFieldSearchAfterLost
        || lastBallPositionField.x() < theFieldDimensions.xPosOwnGroundline + centerSearchStart * 2 * theFieldDimensions.xPosOpponentGroundline)
    {
      for (auto& role : theRoleSelection.selectedRoles)
        if (role == BehaviorData::center || role == BehaviorData::backupBallchaser)
        {
          ballsearch.rolesInBallSearch.push_back(BehaviorData::RoleAssignment(role));
          centerInBallSearch++;
        }
    }
    if (timeSinceBallSearchStarted > timeUntilWholeFieldSearchAfterLost
        || lastBallPositionField.x() >= theFieldDimensions.xPosOwnGroundline + centerSearchStart * 2 * theFieldDimensions.xPosOpponentGroundline)
    {
      for (auto& role : theRoleSelection.selectedRoles)
        if (role == BehaviorData::leftWing || role == BehaviorData::rightWing || role == BehaviorData::receiver)
        {
          ballsearch.rolesInBallSearch.push_back(BehaviorData::RoleAssignment(role));
          offenseInBallSearch++;
        }
    }
  }
  else
    timeStampBallLostForTeam = 0;

  fillBallSearchPositions(ballsearch);
}

void BallSearchProvider::fillBallSearchPositions(BallSearch& ballsearch)
{
  // this method assumes that roles for ball search are already selected!
  for (size_t i = 0; i < ballsearch.rolesInBallSearch.size(); i++)
    ballsearch.ballSearchPositions.push_back(BallSearch::BallSearchPositions());

  int timeSinceBallSearchStarted = theFrameInfo.getTimeSince(timeStampBallLostForTeam);
  switch (defenseInBallSearch)
  {
  case 1:
  {
    for (size_t i = 0; i < ballsearch.rolesInBallSearch.size(); i++)
    {
      const BehaviorData::RoleAssignment& role = ballsearch.rolesInBallSearch[i];
      if (role == BehaviorData::defenderLeft || role == BehaviorData::defenderRight || role == BehaviorData::defenderSingle)
      {
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(135_deg, theFieldDimensions.xPosOwnGroundline + defenseSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline * 0.75f));
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(-135_deg, theFieldDimensions.xPosOwnGroundline + defenseSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline * 0.75f));
      }
    }
    break;
  }
  case 2:
  case 3:
  {
    for (size_t i = 0; i < ballsearch.rolesInBallSearch.size(); i++)
    {
      const BehaviorData::RoleAssignment& role = ballsearch.rolesInBallSearch[i];
      if (role == BehaviorData::defenderLeft)
      {
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(135_deg, theFieldDimensions.xPosOwnGroundline + defenseSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline * 0.8f));
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(-135_deg, theFieldDimensions.xPosOwnGroundline + defenseSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline * 0.35f));
      }
      else if (role == BehaviorData::defenderRight)
      {
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(135_deg, theFieldDimensions.xPosOwnGroundline + defenseSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline * 0.35f));
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(-135_deg, theFieldDimensions.xPosOwnGroundline + defenseSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline * 0.8f));
      }
      else if (role == BehaviorData::defenderSingle)
      {
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(135_deg, theFieldDimensions.xPosOwnGroundline + defenseSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline * 0.25f));
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(-135_deg, theFieldDimensions.xPosOwnGroundline + defenseSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline * 0.25f));
      }
    }
    break;
  }
  default:
    // no one
    break;
  }
  // center positions
  switch (centerInBallSearch)
  {
  case 1:
  {
    for (size_t i = 0; i < ballsearch.rolesInBallSearch.size(); i++)
    {
      const BehaviorData::RoleAssignment& role = ballsearch.rolesInBallSearch[i];
      if (role == BehaviorData::center || role == BehaviorData::backupBallchaser)
      {
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(135_deg, theFieldDimensions.xPosOwnGroundline + centerSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline * 0.75f));
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(-135_deg, theFieldDimensions.xPosOwnGroundline + centerSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline * 0.75f));
      }
    }
    break;
  }
  case 2:
  {
    for (size_t i = 0; i < ballsearch.rolesInBallSearch.size(); i++)
    {
      const BehaviorData::RoleAssignment& role = ballsearch.rolesInBallSearch[i];
      if (role == BehaviorData::center)
      {
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(135_deg, theFieldDimensions.xPosOwnGroundline + centerSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline * 0.8f));
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(-135_deg, theFieldDimensions.xPosOwnGroundline + centerSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline * 0.15f));
      }
      else if (role == BehaviorData::backupBallchaser)
      {
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(135_deg, theFieldDimensions.xPosOwnGroundline + centerSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline * 0.8f));
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(-135_deg, theFieldDimensions.xPosOwnGroundline + centerSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline * 0.15f));
      }
    }
    break;
  }
  default:
    // no one
    break;
  }

  // offensive positions
  switch (offenseInBallSearch)
  {
  case 1:
  {
    for (size_t i = 0; i < ballsearch.rolesInBallSearch.size(); i++)
    {
      const BehaviorData::RoleAssignment& role = ballsearch.rolesInBallSearch[i];
      if (role == BehaviorData::receiver)
      {
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(45_deg, theFieldDimensions.xPosOwnGroundline + (centerSearchStart + 1.f) * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline * 0.75f));
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(-45_deg, theFieldDimensions.xPosOwnGroundline + (centerSearchStart + 1.f) * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline * 0.75f));
        if (centerInBallSearch == 0)
        {
          ballsearch.ballSearchPositions[i].poses.push_back(
              Pose2f(135_deg, theFieldDimensions.xPosOwnGroundline + centerSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline * 0.75f));
          ballsearch.ballSearchPositions[i].poses.push_back(
              Pose2f(-135_deg, theFieldDimensions.xPosOwnGroundline + centerSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline * 0.75f));
        }
      }
    }
    break;
  }
  case 2:
    [[fallthrough]]; // TODO: add 3 positions including rightWing
  case 3:
  {
    for (size_t i = 0; i < ballsearch.rolesInBallSearch.size(); i++)
    {
      const BehaviorData::RoleAssignment& role = ballsearch.rolesInBallSearch[i];
      if (role == BehaviorData::receiver)
      {
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(45_deg, theFieldDimensions.xPosOwnGroundline + (centerSearchStart + 1.f) * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline * 0.8f));
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(-45_deg, theFieldDimensions.xPosOwnGroundline + (centerSearchStart + 1.f) * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline * 0.15f));
        if (centerInBallSearch == 0)
        {
          ballsearch.ballSearchPositions[i].poses.push_back(
              Pose2f(135_deg, theFieldDimensions.xPosOwnGroundline + centerSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline * 0.8f));
          ballsearch.ballSearchPositions[i].poses.push_back(
              Pose2f(-135_deg, theFieldDimensions.xPosOwnGroundline + centerSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline * 0.15f));
        }
      }
      else if (role == BehaviorData::leftWing || role == BehaviorData::rightWing)
      {
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(45_deg, theFieldDimensions.xPosOwnGroundline + (centerSearchStart + 1.f) * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline * 0.8f));
        ballsearch.ballSearchPositions[i].poses.push_back(
            Pose2f(-45_deg, theFieldDimensions.xPosOwnGroundline + (centerSearchStart + 1.f) * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline * 0.15f));
        if (centerInBallSearch == 0)
        {
          ballsearch.ballSearchPositions[i].poses.push_back(
              Pose2f(135_deg, theFieldDimensions.xPosOwnGroundline + centerSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline * 0.8f));
          ballsearch.ballSearchPositions[i].poses.push_back(
              Pose2f(-135_deg, theFieldDimensions.xPosOwnGroundline + centerSearchStart * 2.f * theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline * 0.15f));
        }
      }
    }
    break;
  }
  default:
    // no one
    break;
  }

  // special case: ballchaser goes to last ball position
  if (timeSinceBallSearchStarted < 30000)
  {
    for (size_t i = 0; i < ballsearch.rolesInBallSearch.size(); i++)
    {
      const BehaviorData::RoleAssignment& role = ballsearch.rolesInBallSearch[i];
      if (role == theRoleSymbols.role && theRobotInfo.number == theBallChaserDecision.playerNumberToBall)
      {
        ballsearch.ballSearchPositions[i].poses.clear();
        ballsearch.ballSearchPositions[i].poses.push_back(Pose2f(45_deg,
            std::max(theFieldDimensions.xPosOwnGroundline, std::min(lastBallPositionField.x() + 500.f, theFieldDimensions.xPosOpponentGroundline)),
            std::max(theFieldDimensions.yPosRightSideline, std::min(lastBallPositionField.y() + 1000.f, theFieldDimensions.yPosLeftSideline))));
        ballsearch.ballSearchPositions[i].poses.push_back(Pose2f(-45_deg,
            std::max(theFieldDimensions.xPosOwnGroundline, std::min(lastBallPositionField.x() + 500.f, theFieldDimensions.xPosOpponentGroundline)),
            std::min(theFieldDimensions.yPosLeftSideline, std::max(lastBallPositionField.y() - 1000.f, theFieldDimensions.yPosRightSideline))));
        ballsearch.ballSearchPositions[i].poses.push_back(Pose2f(175_deg,
            std::min(theFieldDimensions.xPosOpponentGroundline, std::max(lastBallPositionField.x() - 1000.f, theFieldDimensions.xPosOwnGroundline)),
            std::max(theFieldDimensions.yPosRightSideline, std::min(lastBallPositionField.y(), theFieldDimensions.yPosLeftSideline))));
      }
    }
  }
}

MAKE_MODULE(BallSearchProvider, behaviorControl)
