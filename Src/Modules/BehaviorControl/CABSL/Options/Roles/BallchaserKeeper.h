option(BallchaserKeeper)
{
  common_transition
  {
    if (theBallSymbols.timeSinceLastSeen > 3000)
    {
      if (theBallSymbols.timeSinceLastSeenByTeamMates < 2000)
        goto lookAtTeamBall;
      else
        goto turnForBall;
    }
    else
      goto ballChase;
  }
  initial_state(ballChase)
  {

    action
    {
      // the ballchaserKeeper behavior is based on critical situations (ball is near own goal = critical; otherwise = not critical)
      // the positioning provider chooses the correct position based for both critical and not critical situations
      // the only reason this role needs a special option is the fact that the AnyKick option is triggered in critical situations
      GoToFieldCoordinates(thePositioningSymbols.optPosition,
          thePositioningSymbols.thresholdXFront,
          thePositioningSymbols.thresholdXBack,
          thePositioningSymbols.thresholdY,
          thePositioningSymbols.thresholdRotation,
          false,
          thePositioningSymbols.previewArrival);

      theBehaviorData.soccerState = BehaviorData::controlBall;
      // walk to the ball and kick it if its in a critical position
      if (gotoFieldCoordinatesFinished && theBallSymbols.ballWasSeen)
        WalkKick(theKeeper.optKickTarget, WalkRequest::any);
    }
  }

  state(lookAtTeamBall)
  {
    action
    {
      GoToFieldCoordinates(Pose2f((theRemoteBallModel.position - theKeeper.optPosition.translation).angle(), thePositioningSymbols.optPosition.translation),
          thePositioningSymbols.thresholdXFront,
          thePositioningSymbols.thresholdXBack,
          thePositioningSymbols.thresholdY,
          thePositioningSymbols.thresholdRotation,
          false,
          thePositioningSymbols.previewArrival);
    }
  }

  state(turnForBall)
  {
    action
    {
      float sign = ((theBallModel.lastPerception.y() > 0) ? 1.f : -1.f);
      Walk(WalkRequest::speed, 0, 0, 60_deg * sign);
      theHeadControlRequest.controlType = HeadControlRequest::direct;
      theHeadControlRequest.pan = 45_deg * sign;
      theHeadControlRequest.tilt = 25_deg;
    }
  }
}