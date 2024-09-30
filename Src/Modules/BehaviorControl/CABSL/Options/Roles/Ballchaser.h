option(Ballchaser)
{

  initial_state(positioning)
  {
    transition
    {
      if (theBallSymbols.ballProbablyCloseButNotSeen)
        goto handleBallLost;
      if (theTacticSymbols.interceptBall)
        goto intercept;
    }
    action
    {
      Positioning(theBallchaser.kickType != MotionRequest::KickType::walkKick, true);
      DecideKick();
    }
  }

  state(handleBallLost)
  {
    transition
    {
      if (theBallSymbols.ballWasSeen || (!theBallSymbols.ballProbablyCloseButNotSeen && state_time > 8500)) // TODO: check
        goto positioning;
    }
    action
    {
      float sign = (theBallSymbols.ballLastSeenLeft ? 1.f : -1.f);
      if (state_time < 2500)
        Walk(WalkRequest::speed, -theWalkingEngineParams.speedLimits.xBackward, sign * 30.f, 0);
      else
      {
        Walk(WalkRequest::speed, 0, 0, 60_deg * sign);
        theHeadControlRequest.controlType = theBallSymbols.ballLastSeenLeft ? HeadControlRequest::ballLostLeft : HeadControlRequest::ballLostRight;
      }
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
}
