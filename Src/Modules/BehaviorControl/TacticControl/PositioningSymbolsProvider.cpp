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
  const auto ballSearchRole = std::find(theBallSearch.rolesInBallSearch.begin(), theBallSearch.rolesInBallSearch.end(), theRoleSymbols.role);
  if (ballSearchRole != theBallSearch.rolesInBallSearch.end())
  {
    const BallSearch::BallSearchPositions& ballSearchPositions = theBallSearch.ballSearchPositions[std::distance(theBallSearch.rolesInBallSearch.begin(), ballSearchRole)];

    if (ballSearchPositions.poses.empty())
    {
      OUTPUT_ERROR("PositioningSymbolsProvider: ballSearchPositions empty");
      return;
    }

    // select closest
    if (!positioningSymbols.inBallSearch && theBallChaserDecision.playerNumberToBall != theRobotInfo.number)
      selectClosestBallSearchPosition(ballSearchPositions, positioningSymbols.optPosition);
    else
    {
      // this can happen if the number of search points varies. in this case, select closest available again
      if (ballSearchPointIndex >= static_cast<int>(ballSearchPositions.poses.size()))
        selectClosestBallSearchPosition(ballSearchPositions, positioningSymbols.optPosition);
      positioningSymbols.optPosition = ballSearchPositions.poses[ballSearchPointIndex];
    }
    // check if target is reached
    if ((theRobotPoseAfterPreview.translation - ballSearchPositions.poses[ballSearchPointIndex].translation).norm() < 100
        && std::abs(Angle::normalize(theRobotPoseAfterPreview.rotation - ballSearchPositions.poses[ballSearchPointIndex].rotation)) < 10_deg)
    {
      // select next point
      ballSearchPointIndex++;
      // if last point was reached, reset to first point
      if (ballSearchPointIndex >= static_cast<int>(ballSearchPositions.poses.size()))
        ballSearchPointIndex = 0;
    }

    // set thresholds and arrival type
    positioningSymbols.thresholdXBack = 30.f;
    positioningSymbols.thresholdXFront = 30.f;
    positioningSymbols.thresholdY = 30.f;
    positioningSymbols.thresholdRotation = 5_deg;
    positioningSymbols.stopAtTarget = false;
    positioningSymbols.previewArrival = true;

    positioningSymbols.inBallSearch = true;
    return;
  }
  else
  {
    positioningSymbols.inBallSearch = false;
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
  case BehaviorData::remoteControl:
    positioningSymbols = theRemoteControl;
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
  static_assert(BehaviorData::RoleAssignment::numOfRoleAssignments == 14, "Missing role!");

  positioningSymbols.distanceToOptPosition = (positioningSymbols.optPosition.translation - theRobotPoseAfterPreview.translation).norm();

  bool inOpponentHalf = theRobotPoseAfterPreview.translation.x() > theFieldDimensions.xPosHalfWayLine - 200.f && theGameInfo.state == STATE_READY && theGameInfo.setPlay == SET_PLAY_NONE;

  bool notOnField = theRobotPoseAfterPreview.translation.x() < theFieldDimensions.xPosOwnGroundline
      || theRobotPoseAfterPreview.translation.y() > theFieldDimensions.yPosLeftSideline || theRobotPoseAfterPreview.translation.y() < theFieldDimensions.yPosRightSideline;
  bool illegalyInCenterCircle = !theGameSymbols.ownKickOff && (theRobotPoseAfterPreview.translation - Vector2f(0.f, 0.f)).norm() < theFieldDimensions.centerCircleRadius + 200.f;
  positioningSymbols.inIllegalPosition = inOpponentHalf || notOnField || illegalyInCenterCircle;
}

void PositioningSymbolsProvider::update(PositioningAndKickSymbols& positioningAndKickSymbols)
{
  switch (theRoleSymbols.role)
  {
  case BehaviorData::remoteControl:
    positioningAndKickSymbols = theRemoteControl;
    break;
  default: //Ballchaser
    positioningAndKickSymbols = theBallchaser;
    break;
  }

  // TODO Add Keeper
  // positioningAndKickSymbols = theKeeper;
}

void PositioningSymbolsProvider::selectClosestBallSearchPosition(const BallSearch::BallSearchPositions& ballSearchPositions, Pose2f& position)
{
  float closestDistance = std::numeric_limits<float>::max();
  int ballPointIndex = 0;
  for (const Pose2f& pose : ballSearchPositions.poses)
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
