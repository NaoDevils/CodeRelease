#include "HeadPOIListGenerator.h"


// This registers your module and specifies a category that assigns it to
// an appropriate cycle (motion or cognition). (see Category enum in Module.h)
MAKE_MODULE(HeadPOIListGenerator, behaviorControl);


void HeadPOIListGenerator::update(HeadPOIList& headPOIList)
{
  headPOIList.targets.clear();

  if (theFallDownState.state != FallDownState::State::upright || theRobotInfo.penalty != PENALTY_NONE)
  {
    addStraight(headPOIList);
    return;
  }

  if (theHeadControlRequest.controlType == HeadControlRequest::ControlType::localize)
  {
    addSweep(headPOIList);
    return;
  }

  if ((theGameInfo.state == STATE_READY || theGameInfo.state == STATE_SET || theGameInfo.state == STATE_PLAYING) && theBallChaserDecision.playerNumberToBall == theRobotInfo.number
      && !theBallchaserHeadPOIList.targets.empty() && theRoleSymbols.role != BehaviorData::RoleAssignment::keeper && theRoleSymbols.role != BehaviorData::RoleAssignment::replacementKeeper)
  {
    headPOIList = theBallchaserHeadPOIList;
    return;
  }

  if (theHeadControlRequest.controlType == HeadControlRequest::ControlType::soccer)
  {
    switch (theGameInfo.state)
    {
    case STATE_INITIAL:
    case STATE_STANDBY:
      addStraight(headPOIList);
      break;
    case STATE_READY:
      addSweep(headPOIList);
      break;
    case STATE_SET:
      if (theBallSymbols.ballWasSeen && theRobotPose.validity > 0.9f)
        addBall(headPOIList);
      else
        addSweep(headPOIList);
      break;
    case STATE_PLAYING:
      if (theBehaviorData.playerNumberToBall == theRobotInfo.number && theSpeedInfo.speed == Pose2f(0, Vector2f::Zero()) && theGameSymbols.ownKickOff
          && (theGameInfo.setPlay == SET_PLAY_KICK_IN || theGameInfo.setPlay == SET_PLAY_CORNER_KICK || theGameInfo.setPlay == SET_PLAY_GOAL_KICK || theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK))
      {
        addBallWithSweep(headPOIList);
      }
      else if (theBallSymbols.ballWasSeen)
      {
        if (theMotionRequest.motion == MotionRequest::Motion::walk && theMotionRequest.walkRequest.requestType == WalkRequest::RequestType::destination
            && theMotionRequest.walkRequest.request.translation.norm() > 800 && std::abs(theBallSymbols.ballPositionRelative.angle()) > 90_deg)
        {
          addSweep(headPOIList);
        }
        else if (theBehaviorData.role == BehaviorData::RoleAssignment::keeper || theBehaviorData.role == BehaviorData::RoleAssignment::replacementKeeper
            || theBehaviorData.playerNumberToBall == theRobotInfo.number)
        {
          addBall(headPOIList);
        }
        else
        {
          addBallWithSweep(headPOIList);
        }
      }
      else if (((theBehaviorData.role == BehaviorData::RoleAssignment::keeper && theFrameInfo.getTimeSince(theRemoteBallModel.timeWhenLastSeen) < 5000)
                   || ((theBehaviorData.role == BehaviorData::RoleAssignment::replacementKeeper || theBehaviorData.playerNumberToBall == theRobotInfo.number)
                       && theFrameInfo.getTimeSince(theRemoteBallModel.timeWhenLastSeen) < 3000))
          && theBallSymbols.timeSinceLastSeen > 10000)
      {
        addRemoteBall(headPOIList);
        if (theSpeedInfo.speed.translation.x() > 0.1)
          addStraight(headPOIList);
      }
      else
        addSweep(headPOIList);
      break;
    case STATE_FINISHED:
      addStraight(headPOIList);
      break;
    default:
      break;
    }
  }
  else if (theHeadControlRequest.controlType == HeadControlRequest::ControlType::direct)
  {
    headPOIList.addAngle(Vector2a(theHeadControlRequest.pan, theHeadControlRequest.tilt));
  }
  else if (theHeadControlRequest.controlType == HeadControlRequest::ControlType::ballLostLeft)
  {
    headPOIList.addAngle(ballLostAngles[0]);
  }
  else if (theHeadControlRequest.controlType == HeadControlRequest::ControlType::ballLostRight)
  {
    headPOIList.addAngle(ballLostAngles[1]);
  }
}

void HeadPOIListGenerator::addSweep(HeadPOIList& headPOIList)
{
  if ((theBehaviorData.role == BehaviorData::RoleAssignment::keeper || theBehaviorData.role == BehaviorData::RoleAssignment::replacementKeeper)
      && (theRobotPose.translation.x() < (theFieldDimensions.xPosOwnGoalArea + 100) && std::abs(theRobotPose.translation.y()) < theFieldDimensions.yPosLeftGoalArea - 100))
  {
    headPOIList.addAngle(goalieLocalizeAngles[0]);
    headPOIList.addAngle(goalieLocalizeAngles[1]);
  }
  else
  {
    headPOIList.addAngle(localizeAngles[0]);
    headPOIList.addAngle(localizeAngles[1]);
  }
}

void HeadPOIListGenerator::addBall(HeadPOIList& headPOIList)
{
  headPOIList.addBall();
}

void HeadPOIListGenerator::addRemoteBall(HeadPOIList& headPOIList)
{
  headPOIList.addRemoteBall();
}

void HeadPOIListGenerator::addBallWithSweep(HeadPOIList& headPOIList)
{
  headPOIList.addBall(ballSweepAngles[0]);
  headPOIList.addBall(ballSweepAngles[1]);
}

void HeadPOIListGenerator::addStraight(HeadPOIList& headPOIList)
{
  headPOIList.addAngle(Vector2a::Zero());
}

void HeadPOIListGenerator::addPenaltyCross(HeadPOIList& headPOIList)
{
  headPOIList.addFieldPosition(Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0));
}
