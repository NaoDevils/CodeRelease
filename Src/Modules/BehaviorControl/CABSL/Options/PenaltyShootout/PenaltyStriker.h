/* this is the penalty striker */

option(PenaltyStriker)
{
  const PenaltyShootout2017Parameters::Striker& parameters = theBehaviorConfiguration.penaltyShootout2017Parameters.striker;

  const Angle& rotation = parameters.rotation;

  const float maxXDeviationFront = parameters.maxXDeviationFrontForKick;
  const float maxXDeviationBack = parameters.maxXDeviationBackForKick;
  const float maxYDeviation = parameters.maxYDeviationForKick;
  const Angle maxAngleDeviation = parameters.maxAngleDeviationForKick;

  const float probabilityToKickLeft = parameters.probabilityToKickLeft;

  int remainingTime = theBehaviorConfiguration.penaltyShootout2017Parameters.striker.durationInSecs - theGameSymbols.timeSincePlayingState / 1000;
  if (static_cast<int>(theGameInfo.secsRemaining) > 0 && static_cast<int>(theGameInfo.secsRemaining) < remainingTime)
    remainingTime = static_cast<int>(theGameInfo.secsRemaining);
  const bool timesUp = remainingTime > 0 && remainingTime < parameters.secsToKick;

  Vector2f assumedBallPosition;
  if (theBallSymbols.ballWasSeen && theBallSymbols.ballPositionRelative.norm() < 1500)
  {
    assumedBallPosition = theBallSymbols.ballPositionField;
  }
  else
  {
    assumedBallPosition = Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0);
  }

  /** STATE MACHINE START **/
  initial_state(standInWalk)
  {
    transition
    {
      if (state_time > 2000)
        goto localize;
    }
    action
    {
      Walk(WalkRequest::speed, 0, 0, 0);
    }
  }

  state(localize)
  {
    transition
    {
      if (state_time > (parameters.maxSecsToLocalize * 1000) || theRobotPoseAfterPreview.validity > parameters.positionConfidenceBeforeStarting)
        goto decideKick;
    }
  }

  state(decideKick)
  {
    transition
    {
      if (randomFloat() > probabilityToKickLeft)
        goto prepareKickRightCorner;
      else
        goto prepareKickLeftCorner;
    }
  }

  state(prepareKickLeftCorner)
  {
    // Kick the ball into the left corner of the goal
    Pose2f kickPosition(rotation, assumedBallPosition);
    // First position for kick engine
    kickPosition.translate(-parameters.optDistanceToBallX, (parameters.mirrorKickLeft ? 1 : -1) * parameters.optDistanceToBallY);

    transition
    {
      if (((action_done || timesUp) && state_time > 8000) || (theGameInfo.secsRemaining > 0 && theGameInfo.secsRemaining < 5))
      {
        if (parameters.mirrorKickLeft)
          goto kickRightFoot;
        else
          goto kickLeftFoot;
      }
    }
    action
    {
      GoToFieldCoordinates(kickPosition, maxXDeviationFront, maxXDeviationBack, maxYDeviation, maxAngleDeviation, true, false);
    }
  }

  state(prepareKickRightCorner)
  {
    // Kick the ball into the right corner of the goal
    Pose2f kickPosition(-rotation, assumedBallPosition);
    // First position for kick engine
    kickPosition.translate(-parameters.optDistanceToBallX, (parameters.mirrorKickRight ? 1 : -1) * parameters.optDistanceToBallY);

    transition
    {
      if (((action_done || timesUp) && state_time > 8000) || (theGameInfo.secsRemaining > 0 && theGameInfo.secsRemaining < 5))
      {
        if (parameters.mirrorKickRight)
          goto kickRightFoot;
        else
          goto kickLeftFoot;
      }
    }
    action
    {
      GoToFieldCoordinates(kickPosition, maxXDeviationFront, maxXDeviationBack, maxYDeviation, maxAngleDeviation, true, false);
    }
  }

  state(kickLeftFoot)
  {
    // Kick target relative to the robots coordinate system
    const Vector2f kickTarget(20000, 0);

    transition
    {
      if (state_time > 5000 || action_done)
        goto finish;
    }
    action
    {
      // left kick (unmirrored)
      LongKick(false, false, kickTarget, parameters.kickIdKickLeft);
    }
  }

  state(kickRightFoot)
  {
    // Kick target relative to the robots coordinate system
    const Vector2f kickTarget(20000, 0);

    transition
    {
      if (state_time > 5000 || action_done)
        goto finish;
    }
    action
    {
      // mirrored left kick -> right kick
      LongKick(true, false, kickTarget, parameters.kickIdKickLeft);
    }
  }

  target_state(finish)
  {
    action
    {
      if (state_time < 3000)
        SpecialAction(SpecialActionRequest::stand);
      else
        SpecialAction(SpecialActionRequest::sitDown);
    }
  }
}
