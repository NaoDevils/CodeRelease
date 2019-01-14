option(AnyKick, (Vector2f) kickTarget)
{
  initial_state(kick)
  {
    action
    {
      theMotionRequest.walkRequest.stepRequest = WalkRequest::StepRequest::any;
      theMotionRequest.kickRequest.kickTarget = kickTarget;
    }
  }
}