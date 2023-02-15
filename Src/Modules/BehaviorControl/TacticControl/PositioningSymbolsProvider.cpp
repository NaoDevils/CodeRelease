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
      if (!wasInBallSearch && theRoleSymbols.role != BehaviorData::ballchaser)
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
    positioningSymbols.optPosition = theKeeper.optPosition;
    positioningSymbols.thresholdXBack = theKeeper.thresholdXBack;
    positioningSymbols.thresholdXFront = theKeeper.thresholdXFront;
    positioningSymbols.thresholdY = theKeeper.thresholdY;
    positioningSymbols.thresholdRotation = theKeeper.thresholdRotation;
    positioningSymbols.stopAtTarget = theKeeper.stopAtTarget;
    positioningSymbols.previewArrival = theKeeper.previewArrival;
    break;
  case BehaviorData::defenderRight:
    positioningSymbols.optPosition = theDefenderRight.optPosition;
    positioningSymbols.thresholdXBack = theDefenderRight.thresholdXBack;
    positioningSymbols.thresholdXFront = theDefenderRight.thresholdXFront;
    positioningSymbols.thresholdY = theDefenderRight.thresholdY;
    positioningSymbols.thresholdRotation = theDefenderRight.thresholdRotation;
    positioningSymbols.stopAtTarget = theDefenderRight.stopAtTarget;
    positioningSymbols.previewArrival = theDefenderRight.previewArrival;
    break;
  case BehaviorData::defenderLeft:
    positioningSymbols.optPosition = theDefenderLeft.optPosition;
    positioningSymbols.thresholdXBack = theDefenderLeft.thresholdXBack;
    positioningSymbols.thresholdXFront = theDefenderLeft.thresholdXFront;
    positioningSymbols.thresholdY = theDefenderLeft.thresholdY;
    positioningSymbols.thresholdRotation = theDefenderLeft.thresholdRotation;
    positioningSymbols.stopAtTarget = theDefenderLeft.stopAtTarget;
    positioningSymbols.previewArrival = theDefenderLeft.previewArrival;
    break;
  case BehaviorData::defenderSingle:
    positioningSymbols.optPosition = theDefenderSingle.optPosition;
    positioningSymbols.thresholdXBack = theDefenderSingle.thresholdXBack;
    positioningSymbols.thresholdXFront = theDefenderSingle.thresholdXFront;
    positioningSymbols.thresholdY = theDefenderSingle.thresholdY;
    positioningSymbols.thresholdRotation = theDefenderSingle.thresholdRotation;
    positioningSymbols.stopAtTarget = theDefenderSingle.stopAtTarget;
    positioningSymbols.previewArrival = theDefenderSingle.previewArrival;
    break;
  case BehaviorData::center:
    positioningSymbols.optPosition = theCenter.optPosition;
    positioningSymbols.thresholdXBack = theCenter.thresholdXBack;
    positioningSymbols.thresholdXFront = theCenter.thresholdXFront;
    positioningSymbols.thresholdY = theCenter.thresholdY;
    positioningSymbols.thresholdRotation = theCenter.thresholdRotation;
    positioningSymbols.stopAtTarget = theCenter.stopAtTarget;
    positioningSymbols.previewArrival = theCenter.previewArrival;
    break;
  case BehaviorData::receiver:
    positioningSymbols.optPosition = theReceiver.optPosition;
    positioningSymbols.thresholdXBack = theReceiver.thresholdXBack;
    positioningSymbols.thresholdXFront = theReceiver.thresholdXFront;
    positioningSymbols.thresholdY = theReceiver.thresholdY;
    positioningSymbols.thresholdRotation = theReceiver.thresholdRotation;
    positioningSymbols.stopAtTarget = theReceiver.stopAtTarget;
    positioningSymbols.previewArrival = theReceiver.previewArrival;
    break;
  case BehaviorData::replacementKeeper:
    positioningSymbols.optPosition = theReplacementKeeper.optPosition;
    positioningSymbols.thresholdXBack = theReplacementKeeper.thresholdXBack;
    positioningSymbols.thresholdXFront = theReplacementKeeper.thresholdXFront;
    positioningSymbols.thresholdY = theReplacementKeeper.thresholdY;
    positioningSymbols.thresholdRotation = theReplacementKeeper.thresholdRotation;
    positioningSymbols.stopAtTarget = theReplacementKeeper.stopAtTarget;
    positioningSymbols.previewArrival = theReplacementKeeper.previewArrival;
    break;
  case BehaviorData::ballchaserKeeper:
    positioningSymbols.optPosition = theBallchaserKeeper.optPosition;
    positioningSymbols.thresholdXBack = theBallchaserKeeper.thresholdXBack;
    positioningSymbols.thresholdXFront = theBallchaserKeeper.thresholdXFront;
    positioningSymbols.thresholdY = theBallchaserKeeper.thresholdY;
    positioningSymbols.thresholdRotation = theBallchaserKeeper.thresholdRotation;
    positioningSymbols.stopAtTarget = theBallchaserKeeper.stopAtTarget;
    positioningSymbols.previewArrival = theBallchaserKeeper.previewArrival;
    break;
  case BehaviorData::backupBallchaser:
    positioningSymbols.optPosition = theBackupBallchaser.optPosition;
    positioningSymbols.thresholdXBack = theBackupBallchaser.thresholdXBack;
    positioningSymbols.thresholdXFront = theBackupBallchaser.thresholdXFront;
    positioningSymbols.thresholdY = theBackupBallchaser.thresholdY;
    positioningSymbols.thresholdRotation = theBackupBallchaser.thresholdRotation;
    positioningSymbols.stopAtTarget = theBackupBallchaser.stopAtTarget;
    positioningSymbols.previewArrival = theBackupBallchaser.previewArrival;
    break;
  default: //Ballchaser
    positioningSymbols.optPosition = theBallchaser.optPosition;
    positioningSymbols.thresholdXBack = theBallchaser.thresholdXBack;
    positioningSymbols.thresholdXFront = theBallchaser.thresholdXFront;
    positioningSymbols.thresholdY = theBallchaser.thresholdY;
    positioningSymbols.thresholdRotation = theBallchaser.thresholdRotation;
    positioningSymbols.stopAtTarget = theBallchaser.stopAtTarget;
    positioningSymbols.previewArrival = theBallchaser.previewArrival;
    break;
  }

  positioningSymbols.distanceToOptPosition = (positioningSymbols.optPosition.translation - theRobotPoseAfterPreview.translation).norm();

  bool inOpponentHalf = theRobotPoseAfterPreview.translation.x() > theFieldDimensions.xPosHalfWayLine - 200.f && theGameInfo.state == STATE_READY && theGameInfo.setPlay == SET_PLAY_NONE;

  bool notOnField = theRobotPoseAfterPreview.translation.x() < theFieldDimensions.xPosOwnGroundline
      || theRobotPoseAfterPreview.translation.y() > theFieldDimensions.yPosLeftSideline || theRobotPoseAfterPreview.translation.y() < theFieldDimensions.yPosRightSideline;
  bool illegalyInCenterCircle = !theGameSymbols.ownKickOff && (theRobotPoseAfterPreview.translation - Vector2f(0.f, 0.f)).norm() < theFieldDimensions.centerCircleRadius + 200.f;
  positioningSymbols.inIllegalPosition = inOpponentHalf || notOnField || illegalyInCenterCircle;
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
