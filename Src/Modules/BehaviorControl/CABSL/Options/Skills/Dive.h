option(Dive)
{
  initial_state(decide)
  {
    transition
    {
      bool isPenaltyKick = theGameSymbols.currentSetPlay == SET_PLAY_PENALTY_KICK || theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT
          || (theGameInfo.state == STATE_PLAYING && theBehaviorConfiguration.behaviorParameters.goalieForEvents);

      if (isPenaltyKick)
      {
        if (std::abs(theBallSymbols.yPosWhenBallReachesOwnYAxis) > 180)
          goto penalty;
      }
      else
      {
        if (theBehaviorConfiguration.behaviorParameters.useDive && std::abs(theBallSymbols.yPosWhenBallReachesOwnYAxis) > 300
            && std::abs(theRobotPoseAfterPreview.rotation) < 30_deg && theRobotInfo.number == 1)
          goto dive;
        //else
        //goto block;
      }
    }
    action {}
  }

  state(dive)
  {
    transition
    {
      if (state_time > 2000 || theMotionInfo.inBlockMotion())
      {
        if (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
          goto waitAfterDive;
        else
          goto finished;
      }
    }
    action
    {
      if (theBallSymbols.yPosWhenBallReachesOwnYAxis > 0)
      {
        if (theBehaviorConfiguration.behaviorParameters.behaviorTestmode) // Use save motions for testing
        {
          if (state_time == 0)
            SystemCall::text2Speech("Defend left");
        }
        else
        {
          SpecialAction(SpecialActionRequest::goalkeeperDefendLeft, false);
          theMotionRequest.GoalieIsDiving = true;
        }
      }
      else
      {
        if (theBehaviorConfiguration.behaviorParameters.behaviorTestmode) // Use save motions for testing
        {
          if (state_time == 0)
            SystemCall::text2Speech("Defend right");
        }
        else
        {
          SpecialAction(SpecialActionRequest::goalkeeperDefendLeft, true);
          theMotionRequest.GoalieIsDiving = true;
        }
      }
    }
  }

  /*state(block)
   {
      transition
      {
        if ((theBallSymbols.ballPositionRelativePredicted.x() > 0.f && state_time > 1000) || state_time > 2000)
        {
          goto finished;
        }
      }
      action
      {
        if(theBehaviorConfiguration.behaviorParameters.behaviorTestmode) // Use save motions for testing
        {
          if (state_time == 0)
            SystemCall::text2Speech("Wide stance");
        }
        else
        {
          SpecialAction(SpecialActionRequest::wideStanceWithStandUp, false);
        }
        theHeadControlRequest.controlType = HeadControlRequest::direct;
        theHeadControlRequest.pan = 0_deg;
        theHeadControlRequest.tilt = 30_deg;

      }
   }*/

  state(penalty)
  {
    transition
    {
      if (state_time > 2000 || theMotionInfo.inBlockMotion())
        goto finished;
    }
    action
    {
      if (theBallSymbols.yPosWhenBallReachesOwnYAxis > 0)
      {
        if (theBehaviorConfiguration.behaviorParameters.behaviorTestmode) // Use save motions for testing
        {
          if (state_time == 0)
            SystemCall::text2Speech("Penalty left");
        }
        else if (theMotionInfo.motion == MotionRequest::Motion::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::SpecialActionID::penaltyGoaliePrepareDive)
        {
          SpecialAction(SpecialActionRequest::penaltyGoalieDiveLeft, false);
          theMotionRequest.GoalieIsDiving = true;
        }
        else
        {
          SpecialAction(SpecialActionRequest::goalkeeperDefendLeft, false);
          theMotionRequest.GoalieIsDiving = true;
        }
      }
      else
      {
        if (theBehaviorConfiguration.behaviorParameters.behaviorTestmode) // Use save motions for testing
        {
          if (state_time == 0)
            SystemCall::text2Speech("Penalty right");
        }
        else if (theMotionInfo.motion == MotionRequest::Motion::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::SpecialActionID::penaltyGoaliePrepareDive)
        {
          SpecialAction(SpecialActionRequest::penaltyGoalieDiveLeft, true);
          theMotionRequest.GoalieIsDiving = true;
        }
        else
        {
          SpecialAction(SpecialActionRequest::goalkeeperDefendLeft, true);
          theMotionRequest.GoalieIsDiving = true;
        }
      }
    }
  }

  state(waitAfterDive)
  {
    action
    {
      // just wait until penalty shot is over
    }
  }

  target_state(finished)
  {
    action
    {
      Walk(WalkRequest::speed, 0, 0, 0);
    }
  }
}
