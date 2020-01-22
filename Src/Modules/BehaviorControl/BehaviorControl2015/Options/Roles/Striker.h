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
      if (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > 5000)
        goto lookForBall;
    }
      action
    {
      const Vector2f ballPositionField = Transformation::robotToField(theRobotPoseAfterPreview,theBallModel.estimate.position);
      Pose2f optKickPosition(
        (Vector2f(theFieldDimensions.xPosOpponentGroundline,theFieldDimensions.yPosCenterGoal) - ballPositionField).angle(),
        ballPositionField
      );
      GoToFieldCoordinates(optKickPosition, 50, 50, 30, 10, false, false);
      AnyKick(Vector2f(theFieldDimensions.xPosOpponentGoal, theFieldDimensions.yPosCenterGoal));
    }
  }

}
