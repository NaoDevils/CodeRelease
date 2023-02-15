option(LongKick, 
  (bool) mirror, (bool) dynamical, (Vector2f) target, ((KickRequest) KickMotionID) kickID)
{
  initial_state(kick)
  {
    transition
    {
      if (theMotionSelection.ratios[MotionRequest::kick] == 1.f || state_time > 5000)
        goto kickInProgress;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::kick;
      theMotionRequest.kickRequest.kickMotionType = kickID;
      theMotionRequest.kickRequest.dynamical = dynamical;
      theMotionRequest.kickRequest.mirror = mirror;
      theMotionRequest.kickRequest.kickTarget = target;
    }
  }

  state(kickInProgress)
  {

    transition
    {
      if (state_time > 1000)
        goto kick_done;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::kick;
      theMotionRequest.kickRequest.kickMotionType = kickID;
      theMotionRequest.kickRequest.dynamical = dynamical;
      theMotionRequest.kickRequest.mirror = mirror;
      theMotionRequest.kickRequest.kickTarget = target;
    }
  }

  target_state(kick_done)
  {
    if (theMotionInfo.motion == MotionRequest::kick)
      timeStampLastLongKickExecution = theFrameInfo.time;
    action
    {
      Walk(WalkRequest::speed, 0, 0, 0);
    }
  }
}
