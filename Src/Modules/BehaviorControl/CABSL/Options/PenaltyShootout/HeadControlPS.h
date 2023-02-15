option(HeadControlPS)
{

  common_transition
  {
    if (theRobotInfo.penalty != PENALTY_NONE)
      goto penalized;
    else
      goto defaultMode;
  }

  initial_state(defaultMode)
  {
    action
    {
      if (theRoleSymbols.role == BehaviorData::keeper)
      {
        theHeadControlRequest.controlType = HeadControlRequest::direct;
        theHeadControlRequest.tilt = 18_deg;
        theHeadControlRequest.pan = 0;
      }
      else
      {
        if (theBallSymbols.timeSinceLastSeen > 2000)
        {
          theHeadControlRequest.controlType = HeadControlRequest::soccer;
        }
        else
        {
          theHeadControlRequest.controlType = HeadControlRequest::ball;
        }
      }
    }
  }

  state(penalized)
  {
    theHeadControlRequest.controlType = HeadControlRequest::direct;
    theHeadControlRequest.tilt = 0.3f;
    theHeadControlRequest.pan = 0;
  }
}
