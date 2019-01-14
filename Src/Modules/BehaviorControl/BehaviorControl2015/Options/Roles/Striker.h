option(Striker)
{
  /*
  * Behavior of a simple striker.
  */
  initial_state(lookForBall)
  {
    transition
    {
      if (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 2000)
        goto goToBall;
    }
    action
    {
      Walk(WalkRequest::speed, 0, 0, 0);
    }
  }

  state(goToBall)
  {
    transition
    {
      if (action_done && state_time > 2000)
        goto kick;
      else if (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > 5000)
        goto lookForBall;
    }
    action
    {
      Pose2f optKickPosition(Transformation::robotToField(theRobotPoseAfterPreview,theBallModel.estimate.position));
      optKickPosition.rotation = 0.f;
      optKickPosition.translate(-190, -50);
      GoToFieldCoordinates(optKickPosition, 50, 50, 30, 10, false, false);
    }
  }

  state(kick)
  {
    transition
    {
      if (state_time > 500 || (theMotionInfo.motion == MotionRequest::walk && theMotionInfo.walkRequest.stepRequest != WalkRequest::StepRequest::none))
        goto goToBall;
    }
    action
    {
      Walk(WalkRequest::speed, 0, 0, 0, WalkRequest::StepRequest::frontKickShort);
    }
  }

}
