option(ReplacementKeeper)
{
  initial_state(positioning)
  {
    transition
    {
      if (theReplacementKeeper.blockBall)
        goto block;
    }
    action
    {
      Positioning();

      if (theReplacementKeeper.kickIt && gotoFieldCoordinatesFinished)
      {
        WalkKick(Pose2f(thePositioningSymbols.optPosition).translate(1500.f, 0.f).translation, thePositioningSymbols.optPosition, WalkRequest::any);
      }
    }
  }

  state(block)
  {
    transition
    {
      if (state_time > 2000)
        goto positioning;
    }
    action
    {
      SpecialAction(SpecialActionRequest::wideStanceWithStandUp, false);
      theHeadControlRequest.controlType = HeadControlRequest::direct;
      theHeadControlRequest.pan = 0_deg;
      theHeadControlRequest.tilt = 30_deg;
    }
  }
}
