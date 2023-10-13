/**
* @file PositioningSymbolsProvider.cpp
*
* Implementation of class PositioningSymbolsProvider.
*
* @author Janine Frickenschmidt
*/

#include "PositioningSymbolsProvider.h"


void PositioningSymbolsProvider::update(PositioningSymbols& positioningSymbols)
{
  // First check if ball search is active for my role. If it is, use position from my travel point list.
  bool useBallSearchPosition = false;
  ballSearchRoleIndex = 0;
  for (auto& role : theBallSearch.rolesInBallSearch)
  {
    if (role == theRoleSymbols.role)
    {
      useBallSearchPosition = true;
      // select closest
      if (!wasInBallSearch && theBallChaserDecision.playerNumberToBall != theRobotInfo.number)
        selectClosestBallSearchPosition(positioningSymbols.optPosition);
      else
      {
        // this can happen if the number of search points varies. in this case, select closest available again
        if (ballSearchPointIndex >= static_cast<int>(theBallSearch.ballSearchPositions[ballSearchRoleIndex].poses.size()))
          selectClosestBallSearchPosition(positioningSymbols.optPosition);
        positioningSymbols.optPosition = theBallSearch.ballSearchPositions[ballSearchRoleIndex].poses[ballSearchPointIndex];
      }
      // check if target is reached
      if ((theRobotPoseAfterPreview.translation - theBallSearch.ballSearchPositions[ballSearchRoleIndex].poses[ballSearchPointIndex].translation).norm() < 100
          && std::abs(Angle::normalize(theRobotPoseAfterPreview.rotation - theBallSearch.ballSearchPositions[ballSearchRoleIndex].poses[ballSearchPointIndex].rotation)) < 10_deg)
      {
        // select next point
        ballSearchPointIndex++;
        // if last point was reached, reset to first point
        if (ballSearchPointIndex >= static_cast<int>(theBallSearch.ballSearchPositions[ballSearchRoleIndex].poses.size()))
          ballSearchPointIndex = 0;
      }

      wasInBallSearch = true;
    }
    ballSearchRoleIndex++;
  }
  positioningSymbols.inBallSearch = wasInBallSearch;

  if (!useBallSearchPosition)
    wasInBallSearch = false;
  else
  {
    // set thresholds and arrival type
    positioningSymbols.thresholdXBack = 30.f;
    positioningSymbols.thresholdXFront = 30.f;
    positioningSymbols.thresholdY = 30.f;
    positioningSymbols.thresholdRotation = 5_deg;
    positioningSymbols.stopAtTarget = false;
    positioningSymbols.previewArrival = true;
    return;
  }
  // if not in ball search, use position from specific roles
  switch (theRoleSymbols.role)
  {
  case BehaviorData::keeper:
    positioningSymbols = theKeeper;
    break;
  case BehaviorData::defenderRight:
    positioningSymbols = theDefenderRight;
    break;
  case BehaviorData::defenderLeft:
    positioningSymbols = theDefenderLeft;
    break;
  case BehaviorData::defenderSingle:
    positioningSymbols = theDefenderSingle;
    break;
  case BehaviorData::center:
    positioningSymbols = theCenter;
    break;
  case BehaviorData::receiver:
    positioningSymbols = theReceiver;
    break;
  case BehaviorData::replacementKeeper:
    positioningSymbols = theReplacementKeeper;
    break;
  case BehaviorData::leftWing:
    positioningSymbols = theLeftWing;
    break;
  case BehaviorData::rightWing:
    positioningSymbols = theRightWing;
    break;
  case BehaviorData::backWing:
    positioningSymbols = theBackWing;
    break;
  case BehaviorData::frontWing:
    positioningSymbols = theFrontWing;
    break;
  case BehaviorData::backupBallchaser:
    positioningSymbols = theBackupBallchaser;
    break;
  default: //Ballchaser
    positioningSymbols = theBallchaser;
    break;
  }
  static_assert(BehaviorData::RoleAssignment::numOfRoleAssignments == 13, "Missing role!");

  positioningSymbols.distanceToOptPosition = (positioningSymbols.optPosition.translation - theRobotPoseAfterPreview.translation).norm();

  bool inOpponentHalf = theRobotPoseAfterPreview.translation.x() > theFieldDimensions.xPosHalfWayLine - 200.f && theGameInfo.state == STATE_READY && theGameInfo.setPlay == SET_PLAY_NONE;

  bool notOnField = theRobotPoseAfterPreview.translation.x() < theFieldDimensions.xPosOwnGroundline
      || theRobotPoseAfterPreview.translation.y() > theFieldDimensions.yPosLeftSideline || theRobotPoseAfterPreview.translation.y() < theFieldDimensions.yPosRightSideline;
  bool illegalyInCenterCircle = !theGameSymbols.ownKickOff && (theRobotPoseAfterPreview.translation - Vector2f(0.f, 0.f)).norm() < theFieldDimensions.centerCircleRadius + 200.f;
  positioningSymbols.inIllegalPosition = inOpponentHalf || notOnField || illegalyInCenterCircle;
}

void PositioningSymbolsProvider::update(PositioningAndKickSymbols& positioningAndKickSymbols)
{
  positioningAndKickSymbols = theBallchaser;

  // TODO Add Keeper
  // positioningAndKickSymbols = theKeeper;
}

void PositioningSymbolsProvider::selectClosestBallSearchPosition(Pose2f& position)
{
  ASSERT(ballSearchRoleIndex < static_cast<int>(theBallSearch.ballSearchPositions.size()));
  float closestDistance = std::numeric_limits<float>::max();
  int ballPointIndex = 0;
  for (const Pose2f& pose : theBallSearch.ballSearchPositions[ballSearchRoleIndex].poses)
  {
    float distance = (pose.translation - theRobotPoseAfterPreview.translation).norm();
    if (distance < closestDistance)
    {
      closestDistance = distance;
      position = pose;
      ballSearchPointIndex = ballPointIndex;
    }
    ballPointIndex++;
  }
}

MAKE_MODULE(PositioningSymbolsProvider, behaviorControl)
