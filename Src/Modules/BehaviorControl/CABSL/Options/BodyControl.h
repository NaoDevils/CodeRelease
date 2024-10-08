
option(BodyControl)
{
  common_transition
  {
    if (theGameInfo.inPreGame())
      goto initial;
    else if (theGameInfo.state == STATE_FINISHED)
      goto finished;
    else if (theRobotInfo.penalty != PENALTY_NONE)
      goto penalized;
    else if ((theGameInfo.state == STATE_PLAYING || theGameInfo.state == STATE_READY) // In state ready or playing,
        && theFallDownState.state == FallDownState::upright // upright,
        && theKeySymbols.obstacle_hit // obstacle was hit
        && !(theBallSymbols.ballInOwnPenaltyArea && theRoleSymbols.role == BehaviorData::keeper)) // and not the goalkeeper in his penalty area
      goto goBack;
    else if (lastGameState == STATE_INITIAL && theGameInfo.state == STATE_READY)
    {
      theHeadControlRequest.controlType = HeadControlRequest::localize;
      goto initialWalkIn;
    }
    else if (theGameInfo.state == STATE_READY)
      goto ready;
    else if (theGameInfo.state == STATE_SET)
      goto set;
    else if (theGameInfo.state == STATE_PLAYING)
      goto playing;
  }

  initial_state(initial)
  {
    transition {}
    action
    {
      lastGameState = STATE_INITIAL;
      if (theVisualRefereeBehaviorSymbols.state != VisualRefereeBehaviorSymbols::State::idle)
      {
        Stand();

        if (theVisualRefereeBehaviorSymbols.state == VisualRefereeBehaviorSymbols::State::localize)
          theHeadControlRequest.controlType = HeadControlRequest::localize;
        else
        {
          theHeadControlRequest.controlType = HeadControlRequest::direct;
          theHeadControlRequest.pan = theVisualRefereeBehaviorSymbols.targetHeadAngle.x();
          theHeadControlRequest.tilt = theVisualRefereeBehaviorSymbols.targetHeadAngle.y();
        }
      }
      else
        SpecialAction(SpecialActionRequest::sitDown);
    }
  }

  state(penalized)
  {
    transition {}
    action
    {
      if (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
        SpecialAction(SpecialActionRequest::sitDown);
      else
        Stand();
    }
  }

  state(initialWalkIn)
  {
    transition {}
    action
    {
      // This triggers the transition to state ready after the initialWalkInTime. Needs to be in action, because of common transition.
      if (state_time > theBehaviorConfiguration.behaviorParameters.initialWalkInTime || theRobotPose.validity > theBehaviorConfiguration.behaviorParameters.initialWalkInMinValidity)
        lastGameState = STATE_READY;

      Walk(WalkRequest::speed, theBehaviorConfiguration.behaviorParameters.initialWalkInSpeed, 0, 0, WalkRequest::none);
    }
  }

  state(ready)
  {
    transition {}
    action
    {
      theBehaviorData.soccerState = BehaviorData::positioning;
      Positioning(false,
          theBallChaserDecision.playerNumberToBall == theRobotInfo.number && theRoleSymbols.role != BehaviorData::RoleAssignment::keeper
              && theRoleSymbols.role != BehaviorData::RoleAssignment::replacementKeeper);
    }
  }

  state(set)
  {
    transition {}
    action
    {
      lastGameState = STATE_SET;
      if (theRoleSymbols.role == BehaviorData::RoleAssignment::keeper && (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK || theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
          && !theGameSymbols.ownKickOff)
        SpecialAction(SpecialActionRequest::SpecialActionID::penaltyGoaliePrepareDive);
      else
        Stand();
    }
  }

  state(playing)
  {
    transition {}
    action
    {
      if (theBehaviorData.behaviorState == BehaviorData::testingJoints)
        theBehaviorData.behaviorState = BehaviorData::game;
      // monitor walk kick executions
      if (theMotionInfo.customStepKickInPreview)
      {
        timeStampLastWalkKickExecution = theFrameInfo.time;
      }

      // Our defense does not move instantly if not alone, to make sure no false whistle was detected (WhistleHandlerDortmund changes state back)
      // This give the ref/gc 10 seconds to apply penalty, otherwise this does not help
      if (theGameSymbols.timeSincePlayingState < 10000 && theGameSymbols.lastGameState == STATE_SET && theTeammateData.numberOfActiveTeammates > 1
          && (theRoleSymbols.role == BehaviorData::keeper || theRoleSymbols.role == BehaviorData::replacementKeeper || theRoleSymbols.role == BehaviorData::defenderLeft
              || theRoleSymbols.role == BehaviorData::defenderRight || theRoleSymbols.role == BehaviorData::defenderSingle || theRoleSymbols.role == BehaviorData::backWing)
          && (theRobotPose.translation - thePositioningSymbols.optPosition.translation).norm() < 400.f && theGameSymbols.kickoffInProgress
          && !(theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT || theGameInfo.setPlay == SET_PLAY_PENALTY_KICK))
      {
        Walk(WalkRequest::speed, 0, 0, 0);
      }
      else if (theGameSymbols.timeSinceLastPenalty < 10000 && theRobotPose.validity < 0.7f) // Initially theGameSymbols.timeSinceLastPenalty = 100000.
      {
        theHeadControlRequest.controlType = HeadControlRequest::localize;
        Walk(WalkRequest::speed, 0, 0, 0);
      }
      else // Actual game play starts here
      {
        lastGameState = STATE_PLAYING;
        Playing();
      }
    }
  }

  state(finished)
  {
    transition {}
    action
    {
      lastGameState = STATE_FINISHED;
      Cheering(); // If the game was won, a random cheering motion will be executed
    }
  }

  state(goBack)
  {
    transition {}
    action
    {
      float dir = 0;
      if (theBallSymbols.ballWasSeen && theBallSymbols.ballPositionRelative.norm() < 300)
        dir = (float)sgn(theBallSymbols.ballPositionRelative.y()) * 60;
      Walk(WalkRequest::speed, -theWalkingEngineParams.speedLimits.xBackward * 0.75f, dir, 0);
    }
  }
}
