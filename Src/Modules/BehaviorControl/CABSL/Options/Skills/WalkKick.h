option(WalkKick, (Vector2f) kickTarget, ((WalkRequest) StepRequest)(WalkRequest::StepRequest::none) stepRequest)
{
  initial_state(kick)
  {
    action
    {
      theMotionRequest.walkRequest.stepRequest = stepRequest;
      theMotionRequest.kickRequest.kickTarget = kickTarget;
    }
  }
}