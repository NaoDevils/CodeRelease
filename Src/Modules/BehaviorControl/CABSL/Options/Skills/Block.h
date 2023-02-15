option(Block)
{
  initial_state(block)
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
      if (theBehaviorConfiguration.behaviorParameters.behaviorTestmode) // Use save motions for testing
      {
        SpecialAction(SpecialActionRequest::sitDown);
      }
      else
      {
        SpecialAction(SpecialActionRequest::wideStanceWithStandUp, false);
      }
      theHeadControlRequest.controlType = HeadControlRequest::direct;
      theHeadControlRequest.pan = 0_deg;
      theHeadControlRequest.tilt = 30_deg;
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