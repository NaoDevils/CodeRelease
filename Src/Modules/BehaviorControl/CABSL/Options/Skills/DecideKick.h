option(DecideKick)
{
  common_transition
  {
    const bool arrivedAtTarget = gotoFieldCoordinatesFinished;
    const bool seesBall = theBallSymbols.ballWasSeen;
    const bool mayKickBlindAndHasNotKickedRecently = thePositioningAndKickSymbols.kickBlind && theFrameInfo.time > (theSpeedInfo.lastCustomStepTimestamp + 500) && theBallModel.validity > 0.f;

    if (arrivedAtTarget && (seesBall || mayKickBlindAndHasNotKickedRecently))
    {
      const bool dribbleSelected = thePositioningAndKickSymbols.kickType == MotionRequest::dribble;
      const bool walkKickSelected = thePositioningAndKickSymbols.kickType == MotionRequest::walkKick && thePositioningAndKickSymbols.walkKickType != WalkRequest::StepRequest::none;
      const bool longKickSelected = thePositioningAndKickSymbols.kickType == MotionRequest::longKick;

      if (dribbleSelected)
      {
        goto dribble;
      }
      else if (walkKickSelected && theSpeedInfo.stepsSinceLastCustomStep > theBehaviorConfiguration.behaviorParameters.minStepsBetweenWalkKicks)
      {
        goto walkKick;
      }
      else if (longKickSelected)
      {
        goto longKick;
      }
    }
    goto noKickSelected;
  }
  initial_state(noKickSelected)
  {
    transition {}
    action {}
  }
  state(dribble)
  {
    transition {}
    action
    {
      Dribble(thePositioningAndKickSymbols.optPosition);
    }
  }
  state(longKick)
  {
    transition {}
    action
    {
      LongKick(theBallSymbols.ballPositionRelative.y() < 0, false, thePositioningAndKickSymbols.kickTarget, thePositioningAndKickSymbols.longKickType);
    }
  }
  state(walkKick)
  {
    transition {}
    action
    {
      WalkKick(thePositioningAndKickSymbols.kickTarget, thePositioningAndKickSymbols.optPosition, thePositioningAndKickSymbols.walkKickType, thePositioningAndKickSymbols.mirrorKick);
    }
  }
}
