option(PerformKickoffWalkKick)
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
          thePositioningSymbols.previewArrival);
    }
  }

  state(executeKick)
  {
    transition
    {
      if (state_time > 800)
        goto walkToBall;
    }
    action
    {
      theMotionRequest.walkRequest.stepRequest = WalkRequest::StepRequest::kickHackLong;
      theMotionRequest.kickRequest.mirror = theTacticSymbols.kickoffToTheLeft;
      theMotionRequest.kickRequest.kickTarget = theBallchaser.kickTarget;
    }
  }
}
