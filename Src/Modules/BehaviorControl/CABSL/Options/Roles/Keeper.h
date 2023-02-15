option(Keeper)
{
  //If ball was last seen close to the goal, then search for ball
  common_transition
  {
    //If ball was last seen close to the goal, then search for ball
    if (theKeeper.ballSearchState != Keeper::KeeperBallSearchState::none && theGameInfo.setPlay != SET_PLAY_PENALTY_KICK && theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT)
    {
      goto search_for_ball;
    }
    // for now with intercept only, since intercept does not have the block state yet
    if (theKeeper.prepared && theBehaviorConfiguration.behaviorParameters.goalieInterceptOnly && theTacticSymbols.interceptBall)
    {
      goto intercept;
    }
    if (theKeeper.catchBall && theKeeper.prepared && theFrameInfo.getTimeSince(theKeeper.timeOfLastDive) > 7000)
    {
      if (theBehaviorConfiguration.behaviorParameters.goalieInterceptOnly)
        goto intercept;
      else
        goto catch_ball;
    }
    else if (theKeeper.isBallchaser)
    {
      goto chase_ball;
    }
  }

  initial_state(positioning)
  {
    transition {}
    action
    {
      //Go to a position relativ to the ball in own penalty area
      if ((theGameInfo.setPlay == SET_PLAY_PENALTY_KICK || theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT) && !theGameSymbols.ownKickOff)
        SpecialAction(SpecialActionRequest::SpecialActionID::penaltyGoaliePrepareDive);
      else
        Positioning();
      // Ingmar, 15.07.2020: RotationType towardsBall was used here, has to be reimplemented if it is ever to be used.
      theBehaviorData.soccerState = BehaviorData::positioning;
    }
  }

  state(intercept)
  {
    transition
    {
      if (!theTacticSymbols.interceptBall)
        goto positioning;
    }
    action
    {
      InterceptBall(false);
    }
  }

  state(catch_ball)
  {
    transition
    {
      if (action_done && !theMotionInfo.inBlockMotion())
      {
        goto positioning;
      }
    }
    action
    {
      Dive();
      theBehaviorData.soccerState = BehaviorData::standUp;
    }
  }
  //how to search for ball
  state(search_for_ball)
  {
    transition
    {
      if (theKeeper.ballSearchState == Keeper::KeeperBallSearchState::none)
      {
        goto positioning;
      }
    }
    action
    {
      KeeperBallSearch();
      theBehaviorData.soccerState = BehaviorData::searchForBall;
    }
  }

  state(chase_ball)
  {
    transition
    {
      if (!theKeeper.isBallchaser)
        goto positioning;
    }

    action
    {
      Positioning(theKeeper.useLongKick);
      if (action_done)
      {
        theMotionRequest.kickRequest.kickTarget = theKeeper.optKickTarget;
        theMotionRequest.kickRequest.mirror = theBallSymbols.ballPositionRelative.y() < 0;
        if (!theKeeper.useLongKick)
        {
          theMotionRequest.walkRequest.stepRequest = theKeeper.walkKick;
        }
        else
        {
          theMotionRequest.motion = MotionRequest::kick;
          theMotionRequest.kickRequest.kickMotionType = theKeeper.longKick;
        }
      }
      theBehaviorData.soccerState = BehaviorData::controlBall;
    }
  }
}
