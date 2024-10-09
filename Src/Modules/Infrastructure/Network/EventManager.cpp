#include "EventManager.h"

MAKE_MODULE(EventManager, cognitionInfrastructure);

const EventManagerBase::Parameters::EventInterval* EventManagerBase::Parameters::getInterval(TeamCommEvents::SendReason reason, bool perTeam) const
{
  const auto it = std::find_if(eventIntervals.begin(),
      eventIntervals.end(),
      [=](const EventInterval& interval)
      {
        return interval.reason == reason && interval.perTeam == perTeam;
      });
  return it != eventIntervals.end() ? &*it : nullptr;
}

void EventManager::update(TeamCommEvents& events)
{
  events.sendReasons = getSendReasons();

  //const Teammate* newestTeammate = theTeammateData.getNewestTeammate();

  //const float messageBudgetFactor = std::min(1.f, 0.9f + static_cast<float>(theFrameInfo.getTimeSince(theGameInfo.timeFirstReadyState)) / 900000.f);

  const bool isPenalized = theRobotInfo.penalty != PENALTY_NONE;
  const bool eventOldEnough = isEventOldEnough(events.sendReasons);

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

  //const bool inMessageBudget = theTeammateData.messageBudgetFactor >= messageBudgetFactor || !newestTeammate
  //    || theFrameInfo.getTimeSince(newestTeammate->sendTimestamp) > static_cast<int>(globalSendLimitWhenMessageRateTooHigh);

  const bool allowedToSend = theTeammateData.messageBudget >= 5 // 5 message buffer for safety
      && theRobotInfo.transitionToFramework == 1.f // framework active
      && theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT;

  events.sendThisFrame = active && allowedToSend /* && inMessageBudget */;

  if (events.sendThisFrame)
    for (const auto sendReason : events.sendReasons)
      newestLocalUpdates[sendReason].push_back(theFrameInfo.time);

  for (int reason = 0; reason < TeamCommEvents::SendReason::numOfSendReasons; ++reason)
  {
    const TeammateReceived* mate = theTeammateData.getNewestEventMessage(static_cast<TeamCommEvents::SendReason>(reason));
    if (mate && (newestTeamUpdates[reason].empty() || mate->sendTimestamp > newestTeamUpdates[reason].back()))
      newestTeamUpdates[reason].push_back(mate->sendTimestamp);
  }

  for (const bool perTeam : {false, true})
  {
    auto& newestUpdates = perTeam ? newestTeamUpdates : newestLocalUpdates;
    for (int reason = 0; reason < TeamCommEvents::SendReason::numOfSendReasons; ++reason)
    {
      const EventInterval* interval = getInterval(static_cast<TeamCommEvents::SendReason>(reason), perTeam);
      while (!newestUpdates[reason].empty() && (!interval || theFrameInfo.getTimeSince(newestUpdates[reason].front()) > static_cast<int>(interval->interval)))
        newestUpdates[reason].pop_front();
    }
  }
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

  if (checkForRefereeGesture())
    sendReasons.push_back(TeamCommEvents::SendReason::refereeGestureDetected);

  if (checkForUprightAgain())
    sendReasons.push_back(TeamCommEvents::SendReason::uprightAgain);


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
  const bool positionUpdateInitial = theGameInfo.inPreGame() && theFrameInfo.getTimeSince(theTeamCommSenderOutput.dataSentTimestamp) > static_cast<int>(eventConfig.playerMovedEventIntervalInitial);

  if (theTeamCommSenderOutput.dataSent)
    lastSendPosition = theRobotPose;

  // do not send position updates after 7s in ready
  if (theTacticSymbols.keepRoleAssignment)
    return false;

  const float distanceMoved = (theRobotPose.translation - lastSendPosition.translation).norm();
  const float ballDistance = theBehaviorData.ballPositionRelative.norm();

  float movedThreshold;
  if (theSpeedInfo.speed.translation.norm() * 1000.f < eventConfig.playerMovedEventSlowTransSpeed && std::abs(theSpeedInfo.speed.rotation) < eventConfig.playerMovedEventSlowRotSpeed)
    movedThreshold = eventConfig.playerMovedEventDistanceWhenSlow;
  else if (theBehaviorData.playerNumberToBall == theRobotInfo.number)
    movedThreshold = eventConfig.playerMovedEventDistanceForBallchaser;
  else if (ballDistance < eventConfig.playerMovedNearBallDistance)
    movedThreshold = eventConfig.playerMovedEventDistanceForNearBall;
  else
    movedThreshold = eventConfig.playerMovedEventDistance;

  Vector2f goalAreaBottomLeft(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoalArea);
  Vector2f goalAreaTopRight(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea);
  const bool positionUpdateByAllowedInGoalArea = Geometry::isPointInsideRectangle(goalAreaBottomLeft, goalAreaTopRight, theTeammateData.myself.robotPose.translation)
          != Geometry::isPointInsideRectangle(goalAreaBottomLeft, goalAreaTopRight, theRobotPoseAfterPreview.translation)
      && theFrameInfo.getTimeSince(theTeamCommSenderOutput.dataSentTimestamp) > static_cast<int>(eventConfig.playerMovedEventIntervalGoalArea) && distanceMoved > 300.f;
  const bool playerMoved = distanceMoved > movedThreshold;

  if ((positionUpdateInitial || playerMoved || positionUpdateByAllowedInGoalArea)
      && (theRobotPose.validity > eventConfig.playerMovedEventMinPoseValidity || theTeammateData.myself.sendTimestamp == 0
          || (theRobotPose.validity > eventConfig.playerMovedEventMinPoseValidityInitial && theGameInfo.inPreGame())))
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
  const TeammateReceived* mate = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::newBallchaser);
  if (mate && mate->behaviorData.playerNumberToBall == theRobotInfo.number && theFallDownState.state != FallDownState::State::upright && !theTeammateData.myself.fallen)
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

bool EventManager::checkForRefereeGesture()
{
  const Teammate* mate = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::refereeGestureDetected);
  if (mate && theFrameInfo.getTimeSince(mate->sendTimestamp) < 30000)
    return false;

  return theRawGameInfo.state == STATE_STANDBY && theVisualRefereeBehaviorSymbols.state == VisualRefereeBehaviorSymbols::State::capture
      && theRefereeGesture.gesture != RefereeGesture::Gesture::NONE && theRawGameInfo.controllerConnected;
}

bool EventManager::checkForUprightAgain()
{
  return theTeammateData.myself.fallen && theFallDownState.state == FallDownState::State::upright;
}


bool EventManager::isEventOldEnough(const std::vector<TeamCommEvents::SendReason>& sendReasons) const
{
  for (const TeamCommEvents::SendReason sendReason : sendReasons)
  {
    const EventInterval* localInterval = getInterval(sendReason, false);
    const EventInterval* teamInterval = getInterval(sendReason, true);
    const bool inLocalInterval = !localInterval || newestLocalUpdates[sendReason].size() < localInterval->limit;
    const bool inTeamInterval = !teamInterval || newestTeamUpdates[sendReason].size() < teamInterval->limit;

    // do not send if number of events is above limit or not configured at all
    if ((inLocalInterval && inTeamInterval) && (localInterval || teamInterval))
      return true;
  }

  return false;
}
