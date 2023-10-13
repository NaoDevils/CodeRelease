option(WalkKick, (Vector2f) kickTarget, (Pose2f) kickPose, ((WalkRequest) StepRequest)(WalkRequest::StepRequest::none) stepRequest, (bool)(false) mirror)
{
  initial_state(kick)
  {
    action
    {
      theMotionRequest.walkRequest.stepRequest = stepRequest;
      theMotionRequest.kickRequest.kickTarget = kickTarget;
      theMotionRequest.kickRequest.kickPose = kickPose;
      theMotionRequest.kickRequest.mirror = mirror;
    }
  }
}