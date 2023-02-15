/** This option lets the robot stand up when it has fallen down. */
option(StandUp)
{
  initial_state(lying)
  {
    transition
    {
      if (theFallDownState.direction == FallDownState::back)
        goto lyingOnBack;
      else if (theFallDownState.direction == FallDownState::front)
        goto lyingOnFront;
      else if (theFallDownState.direction == FallDownState::left || theFallDownState.direction == FallDownState::right)
        goto lyingOnSide;
      else
        goto standing_up;
    }
    action {}
  }

  state(lyingOnBack)
  {
    transition
    {
      if (theMotionInfo.inStandUpMotion())
      {
        goto standing_up;
      }
    }
    action
    {
      if (!theMotionState.standUpStatus.standUp[theMotionSettings.standUpMotionBack])
        SpecialAction(theMotionSettings.standUpMotionBack, false);
      else if (!theMotionState.standUpStatus.standUp[theMotionSettings.standUpMotionBackSafe])
        SpecialAction(theMotionSettings.standUpMotionBackSafe, false);
      else
        SpecialAction(theMotionSettings.standUpMotionBackSafest, false);
    }
  }

  state(lyingOnFront)
  {
    transition
    {
      if (theMotionInfo.inStandUpMotion())
      {
        goto standing_up;
      }
    }
    action
    {
      if (!theMotionState.standUpStatus.standUp[theMotionSettings.standUpMotionFront])
        SpecialAction(theMotionSettings.standUpMotionFront, false);
      else if (!theMotionState.standUpStatus.standUp[theMotionSettings.standUpMotionFrontSafe])
        SpecialAction(theMotionSettings.standUpMotionFrontSafe, false);
      else
        SpecialAction(theMotionSettings.standUpMotionFrontSafest, false);
    }
  }

  state(lyingOnSide)
  {
    transition
    {
      if (state_time > 3000) // TODO: check this
      {
        goto lying;
      }
      if (state_time > 1000)
      {
        if (theFallDownState.direction == FallDownState::back)
          goto lyingOnBack;
        else if (theFallDownState.direction == FallDownState::front)
          goto lyingOnFront;
        else if (theFallDownState.state == FallDownState::upright)
          goto standing_up;
      }
    }
    action
    {
      SpecialAction(SpecialActionRequest::standUpSideNao, theFallDownState.direction == FallDownState::right);
    }
  }

  state(standing_up)
  {
    transition
    {
      if (theMotionInfo.motion != MotionRequest::specialAction && theFallDownState.state == FallDownState::onGround)
      {
        if (theFallDownState.direction == FallDownState::back)
          goto lyingOnBack;
        else if (theFallDownState.direction == FallDownState::front)
          goto lyingOnFront;
        else if (theFallDownState.direction == FallDownState::left || theFallDownState.direction == FallDownState::right)
          goto lyingOnSide;
      }
      else if (theMotionInfo.motion == MotionRequest::walk)
        goto standing;
    }
    action
    {
      Walk(WalkRequest::speed, 0, 0, 0);
    }
  }

  target_state(standing)
  {
    transition
    {
      if (theMotionInfo.motion != MotionRequest::specialAction && theFallDownState.state == FallDownState::onGround)
      {
        if (theFallDownState.direction == FallDownState::back)
          goto lyingOnBack;
        else if (theFallDownState.direction == FallDownState::front)
          goto lyingOnFront;
        else if (theFallDownState.direction == FallDownState::left || theFallDownState.direction == FallDownState::right)
          goto lyingOnSide;
      }
    }
    action
    {
      Walk(WalkRequest::speed, 0, 0, 0);
    }
  }
}
