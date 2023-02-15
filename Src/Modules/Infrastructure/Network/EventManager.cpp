#include "EventManager.h"

MAKE_MODULE(EventManager, cognitionInfrastructure);

void EventManager::update(TeamCommEvents& events)
{
  events.sendReasons.clear();
  checkForSendReasons(events.sendReasons);

  const Teammate* newestTeammate = theTeammateData.getNewestTeammate();

  const float messageBudgetFactor = std::min(1.f, 0.9f + static_cast<float>(theFrameInfo.getTimeSince(theGameInfo.timeFirstReadyState)) / 900000.f);

  events.sendThisFrame = theTeammateData.messageBudget >= 5 // 5 message buffer for safety
      && (isTeamEventOldEnough(events.sendReasons) || isLocalEventOldEnough(events.sendReasons))
      && (theTeammateData.messageBudgetFactor >= messageBudgetFactor || !newestTeammate || theFrameInfo.getTimeSince(newestTeammate->timeWhenSent) > static_cast<int>(globalSendLimitWhenMessageRateTooHigh));

  if (events.sendThisFrame)
    for (const auto sendReason : events.sendReasons)
      newestLocalUpdate[sendReason] = theFrameInfo.time;
}

void EventManager::checkForSendReasons(std::vector<TeamCommEvents::SendReason>& sendReasons)
{
  if (checkForNewRoleAssignment())
    sendReasons.push_back(TeamCommEvents::SendReason::newRolesAssigned);

  if (checkForPlayerMoved())
    sendReasons.push_back(TeamCommEvents::SendReason::playerMoved);

  if (checkForGoalDetected())
    sendReasons.push_back(TeamCommEvents::SendReason::goalDetected);

  if (checkForSymmetryLost())
    sendReasons.push_back(TeamCommEvents::SendReason::symmetryLost);

  if (checkForSymmetryUpdate())
    sendReasons.push_back(TeamCommEvents::SendReason::symmetryUpdate);

  if (checkForBallMoved())
    sendReasons.push_back(TeamCommEvents::SendReason::ballMoved);

  if (checkForBallchaserFallDown())
    sendReasons.push_back(TeamCommEvents::SendReason::ballchaserFallDown);

  if (theGameSymbols.kickoffInProgress == false && wasKickOffInProgress && theRoleSymbols.role == BehaviorData::RoleAssignment::ballchaser)
    sendReasons.push_back(TeamCommEvents::SendReason::kickOffFinished);

  // remember states for next frame
  wasKickOffInProgress = theGameSymbols.kickoffInProgress;
  lastFallDownState = theFallDownState.state;
}

bool EventManager::checkForNewRoleAssignment()
{
  const Teammate* teammate = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::newRolesAssigned);
  const BehaviorData::RoleAssignment myTeamRole = teammate ? teammate->behaviorData.roleSuggestions[theRobotInfo.number] : BehaviorData::RoleAssignment::noRole;
  const bool isOrWasStriker = theBehaviorData.role == BehaviorData::RoleAssignment::ballchaser || myTeamRole == BehaviorData::RoleAssignment::ballchaser;
  const bool isOrWasKeeperBallChaser = theBehaviorData.role == BehaviorData::RoleAssignment::keeper
      && (theBehaviorData.soccerState == BehaviorData::SoccerState::controlBall
          || (teammate && teammate->number == theRobotInfo.number && teammate->behaviorData.soccerState == BehaviorData::SoccerState::controlBall));
  if ((isOrWasStriker || isOrWasKeeperBallChaser) && (!teammate || theBehaviorData.roleSuggestions != teammate->behaviorData.roleSuggestions))
    return true;

  return false;
}

bool EventManager::checkForPlayerMoved()
{
  const bool positionUpdateInitial = theGameInfo.state == STATE_INITIAL
      && theFrameInfo.getTimeSince(theTeamCommSenderOutput.dataSentTimestamp) > static_cast<int>(eventConfig.playerMovedEventIntervalInitial);

  if (theTeamCommSenderOutput.dataSent)
    lastSendPosition = theRobotPose;
  const float distanceMoved = (theRobotPose.translation - lastSendPosition.translation).norm();
  const float ballDistance = theBehaviorData.ballPositionRelative.norm();
  const float movedThreshold = (theBehaviorData.role == BehaviorData::RoleAssignment::ballchaser)
      ? eventConfig.playerMovedEventDistanceForBallchaser
      : ((ballDistance > eventConfig.playerMovedNearBallDistance) ? eventConfig.playerMovedEventDistance : eventConfig.playerMovedEventDistanceForBallchaser);

  Vector2f penAreaBottomLeft(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightPenaltyArea);
  Vector2f penAreaTopRight(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  const bool positionUpdateByAllowedInPenaltyArea = Geometry::isPointInsideRectangle(penAreaBottomLeft, penAreaTopRight, theTeammateData.myself.pose.translation)
          != Geometry::isPointInsideRectangle(penAreaBottomLeft, penAreaTopRight, theRobotPoseAfterPreview.translation)
      && theFrameInfo.getTimeSince(theTeamCommSenderOutput.dataSentTimestamp) > static_cast<int>(eventConfig.playerMovedEventIntervalPenaltyArea) && distanceMoved > 300.f;
  const bool playerMoved = distanceMoved > movedThreshold;

  if ((positionUpdateInitial || playerMoved || positionUpdateByAllowedInPenaltyArea)
      && (theRobotPose.validity > eventConfig.playerMovedEventMinPoseValidity || theTeammateData.myself.timeWhenSent == 0
          || (theRobotPose.validity > eventConfig.playerMovedEventMinPoseValidityInitial && theGameInfo.state == STATE_INITIAL)))
    return true;

  return false;
}

bool EventManager::checkForGoalDetected()
{
  const bool goalDetected = theRawGameInfo.state == STATE_PLAYING && theGameInfo.state == STATE_READY;
  const Teammate* goalDetectedMate = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::goalDetected);
  if (goalDetected && (!goalDetectedMate || theFrameInfo.getTimeSince(goalDetectedMate->timeWhenSent) > static_cast<int>(eventConfig.goalDetectedMinTimeDiff)))
    return true;

  return false;
}

bool EventManager::checkForSymmetryLost()
{
  if (theSideConfidence.confidenceState != SideConfidence::ConfidenceState::CONFIDENT)
    return true;
  else
    return false;
}

bool EventManager::checkForSymmetryUpdate()
{
  const Teammate* teammateSymmetryLost = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::symmetryLost);
  if (teammateSymmetryLost && theFrameInfo.getTimeSince(theTeamCommSenderOutput.dataSentTimestamp) > 500 && teammateSymmetryLost->timeWhenSent > theTeamCommSenderOutput.dataSentTimestamp
      && theBehaviorData.timeSinceBallWasSeen < 1500) // the time since seen has to be in sync with the time used in SelfLocator2017!
    return true;

  return false;
}

bool EventManager::checkForBallMoved()
{
  // only use local model, only if validity is high enough
  if (!theBallSymbols.useLocalBallModel || theBallModel.validity < eventConfig.ballMovedMinValidity)
    return false;
  float minBallDistancePerceptToSent = (theBehaviorData.ballPositionField - theTeammateData.myself.behaviorData.ballPositionField).norm();
  for (const auto& mate : theTeammateData.teammates)
  {
    float distance = (mate.behaviorData.ballPositionField - theBehaviorData.ballPositionField).norm();
    if (distance < minBallDistancePerceptToSent)
      minBallDistancePerceptToSent = distance;
  }
  if (minBallDistancePerceptToSent > eventConfig.ballMovedEventDistance)
    return true;

  return false;
}

bool EventManager::checkForBallchaserFallDown()
{
  if (theRoleSymbols.role == BehaviorData::RoleAssignment::ballchaser && theFallDownState.state != FallDownState::State::upright && lastFallDownState == FallDownState::State::upright)
    return true;
  else
    return false;
}

bool EventManager::isTeamEventOldEnough(const std::vector<TeamCommEvents::SendReason>& sendReasons) const
{
  for (const TeamCommEvents::SendReason sendReason : sendReasons)
  {
    for (const auto& eventInterval : eventIntervals)
    {
      if (eventInterval.reason == sendReason && eventInterval.perTeam)
      {
        const Teammate* teammate = theTeammateData.getNewestEventMessage(sendReason);

        if (!teammate || theFrameInfo.getTimeSince(teammate->timeWhenSent) > static_cast<int>(eventInterval.interval))
          return true;
      }
    }
  }

  return false;
}

bool EventManager::isLocalEventOldEnough(const std::vector<TeamCommEvents::SendReason>& sendReasons) const
{
  for (const TeamCommEvents::SendReason sendReason : sendReasons)
  {
    for (const auto& eventInterval : eventIntervals)
    {
      if (eventInterval.reason == sendReason && !eventInterval.perTeam && theFrameInfo.getTimeSince(newestLocalUpdate[sendReason]) > static_cast<int>(eventInterval.interval))
      {
        return true;
      }
    }
  }

  return false;
}
