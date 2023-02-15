option(DecideKick)
{
  common_transition
  {
    if (theBallchaser.kickType == MotionRequest::dribble)
      goto dribble;
    else if (gotoFieldCoordinatesFinished && theBallSymbols.ballWasSeen)
    {
      if (theBallchaser.kickType == MotionRequest::longKick)
        goto longKick;
      else if (theBallchaser.kickType == MotionRequest::walkKick && theSpeedInfo.stepsSinceLastCustomStep > theBehaviorConfiguration.behaviorParameters.minStepsBetweenWalkKicks
          && theBallchaser.walkKickType != WalkRequest::StepRequest::none)
        goto walkKick;
      else
        goto noKickSelected;
    }
    else
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
      Dribble(theBallchaser.optPosition);
    }
  }
  state(longKick)
  {
    transition {}
    action
    {
      LongKick(theBallSymbols.ballPositionRelative.y() < 0, false, theBallchaser.kickTarget, theBallchaser.longKickType);
    }
  }
  state(walkKick)
  {
    transition {}
    action
    {
      WalkKick(theBallchaser.kickTarget, theBallchaser.walkKickType);
    }
  }
}