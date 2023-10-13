#include "EventManager.h"

MAKE_MODULE(EventManager, cognitionInfrastructure);

void EventManager::update(TeamCommEvents& events)
{
  events.sendReasons = getSendReasons();

  const Teammate* newestTeammate = theTeammateData.getNewestTeammate();

  const float messageBudgetFactor = std::min(1.f, 0.9f + static_cast<float>(theFrameInfo.getTimeSince(theGameInfo.timeFirstReadyState)) / 900000.f);

  const bool isPenalized = theRobotInfo.penalty != PENALTY_NONE;
  const bool eventOldEnough = isTeamEventOldEnough(events.sendReasons) || isLocalEventOldEnough(events.sendReasons);

  bool active = !isPenalized && eventOldEnough;

  // Send one more time when being penalized for ballchaser handover to another player
  // Otherwise, no robot may take over if two players have a similiar distance.
  const Teammate* teammate = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::newBallchaser);
  const bool wasStriker = teammate ? teammate->behaviorData.playerNumberToBall == theRobotInfo.number : true;
  if (isPenalized && wasStriker && checkForNewBallchaser())
  {
    active = true;
    events.sendReasons = {TeamCommEvents::SendReason::newBallchaser};
  }

  const bool inMessageBudget = theTeammateData.messageBudgetFactor >= messageBudgetFactor || !newestTeammate
      || theFrameInfo.getTimeSince(newestTeammate->sendTimestamp) > static_cast<int>(globalSendLimitWhenMessageRateTooHigh);

  const bool allowedToSend = theTeammateData.messageBudget >= 5 // 5 message buffer for safety
      && theRobotInfo.transitionToFramework == 1.f // framework active
      && theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT;

  events.sendThisFrame = active && allowedToSend && inMessageBudget;

  if (events.sendThisFrame)
    for (const auto sendReason : events.sendReasons)
      newestLocalUpdate[sendReason] = theFrameInfo.time;
}

std::vector<TeamCommEvents::SendReason> EventManager::getSendReasons()
{
  std::vector<TeamCommEvents::SendReason> sendReasons;

  if (checkForNewBallchaser())
    sendReasons.push_back(TeamCommEvents::SendReason::newBallchaser);

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

  if (theGameSymbols.kickoffInProgress == false && wasKickOffInProgress && theBehaviorData.playerNumberToBall == theRobotInfo.number)
    sendReasons.push_back(TeamCommEvents::SendReason::kickOffFinished);

  if (checkForTimeResponses())
    sendReasons.push_back(TeamCommEvents::SendReason::timeResponse);


  // remember states for next frame
  wasKickOffInProgress = theGameSymbols.kickoffInProgress;

  return sendReasons;
}

bool EventManager::checkForNewBallchaser()
{
  // do not send static/local ballchaser decision
  if (!theBallChaserDecision.dynamic)
    return false;

  // do not send position updates after 7s in ready
  if (theTacticSymbols.keepRoleAssignment)
    return false;

  const Teammate* teammate = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::newBallchaser);
  const bool wasStriker = teammate ? teammate->behaviorData.playerNumberToBall == theRobotInfo.number : true;
  const bool isOrWasStriker = theBehaviorData.playerNumberToBall == theRobotInfo.number || wasStriker;

  return isOrWasStriker && (!teammate || theBehaviorData.playerNumberToBall != teammate->behaviorData.playerNumberToBall);
}

bool EventManager::checkForNewRoleAssignment()
{
  // do not send static role assignments
  if (!theRoleSymbols.dynamic)
    return false;

  const Teammate* teammate = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::newBallchaser);
  const bool wasStriker = teammate ? teammate->behaviorData.playerNumberToBall == theRobotInfo.number : false;
  const bool isAndWasStriker = theBehaviorData.playerNumberToBall == theRobotInfo.number && wasStriker;

  const Teammate* teammateRole = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::newRolesAssigned);
  if (!teammateRole)
    return true;

  const auto& teammateRoleSuggestions = teammateRole->behaviorData.roleSuggestions;

  return isAndWasStriker && theRoleSymbols.roleSuggestions != teammateRoleSuggestions;
}

bool EventManager::checkForPlayerMoved()
{
  const bool positionUpdateInitial = theGameInfo.state == STATE_INITIAL
      && theFrameInfo.getTimeSince(theTeamCommSenderOutput.dataSentTimestamp) > static_cast<int>(eventConfig.playerMovedEventIntervalInitial);

  if (theTeamCommSenderOutput.dataSent)
    lastSendPosition = theRobotPose;

  // do not send position updates after 7s in ready
  if (theTacticSymbols.keepRoleAssignment)
    return false;

  const float distanceMoved = (theRobotPose.translation - lastSendPosition.translation).norm();
  const float ballDistance = theBehaviorData.ballPositionRelative.norm();
  const float movedThreshold = (theBehaviorData.playerNumberToBall == theRobotInfo.number)
      ? eventConfig.playerMovedEventDistanceForBallchaser
      : ((ballDistance > eventConfig.playerMovedNearBallDistance) ? eventConfig.playerMovedEventDistance : eventConfig.playerMovedEventDistanceForBallchaser);

  Vector2f goalAreaBottomLeft(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoalArea);
  Vector2f goalAreaTopRight(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea);
  const bool positionUpdateByAllowedInGoalArea = Geometry::isPointInsideRectangle(goalAreaBottomLeft, goalAreaTopRight, theTeammateData.myself.robotPose.translation)
          != Geometry::isPointInsideRectangle(goalAreaBottomLeft, goalAreaTopRight, theRobotPoseAfterPreview.translation)
      && theFrameInfo.getTimeSince(theTeamCommSenderOutput.dataSentTimestamp) > static_cast<int>(eventConfig.playerMovedEventIntervalGoalArea) && distanceMoved > 300.f;
  const bool playerMoved = distanceMoved > movedThreshold;

  if ((positionUpdateInitial || playerMoved || positionUpdateByAllowedInGoalArea)
      && (theRobotPose.validity > eventConfig.playerMovedEventMinPoseValidity || theTeammateData.myself.sendTimestamp == 0
          || (theRobotPose.validity > eventConfig.playerMovedEventMinPoseValidityInitial && theGameInfo.state == STATE_INITIAL)))
    return true;

  return false;
}

bool EventManager::checkForGoalDetected()
{
  const bool goalDetected = theRawGameInfo.state == STATE_PLAYING && theGameInfo.state == STATE_READY;
  const Teammate* goalDetectedMate = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::goalDetected);
  if (goalDetected && (!goalDetectedMate || theFrameInfo.getTimeSince(goalDetectedMate->sendTimestamp) > static_cast<int>(eventConfig.goalDetectedMinTimeDiff)))
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
  if (teammateSymmetryLost && theFrameInfo.getTimeSince(theTeamCommSenderOutput.dataSentTimestamp) > 500 && teammateSymmetryLost->sendTimestamp > theTeamCommSenderOutput.dataSentTimestamp
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
  if (theBehaviorData.playerNumberToBall == theRobotInfo.number && theFallDownState.state != FallDownState::State::upright && !theTeammateData.myself.fallen)
    return true;
  else
    return false;
}

bool EventManager::checkForTimeResponses()
{
  // Limit this to a reasonable number of messages
  if (theTeammateData.statistic.events[TeamCommEvents::SendReason::timeResponse] >= 100)
    return false;

  for (const auto& req : theTimeSynchronization.receivedRequests)
  {
    // collect multiple requests over a period of one second
    if (theFrameInfo.getTimeSince(req.received) > 1000)
      return true;
  }
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

        if (!teammate || theFrameInfo.getTimeSince(teammate->sendTimestamp) > static_cast<int>(eventInterval.interval))
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
