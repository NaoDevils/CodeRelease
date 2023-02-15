option(PerformKickoff)
{
  initial_state(walkToBall)
  {
    transition
    {
      if (action_done)
        goto executeKick;
    }
    action
    {
      GoToFieldCoordinates(thePositioningSymbols.optPosition,
          thePositioningSymbols.thresholdXFront,
          thePositioningSymbols.thresholdXBack,
          thePositioningSymbols.thresholdY,
          thePositioningSymbols.thresholdRotation,
          thePositioningSymbols.stopAtTarget,
          false);
    }
  }

  state(executeKick)
  {
    transition
    {
      if (action_done)
        goto walkToBall;
    }
    action
    {
      LongKick(theBallSymbols.ballPositionRelative.y() < 0, false, theBallchaser.kickTarget, theBehaviorConfiguration.behaviorParameters.longKick);
    }
  }
}
