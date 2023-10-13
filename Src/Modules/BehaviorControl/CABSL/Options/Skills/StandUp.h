/** This option lets the robot stand up when it has fallen down. */
option(StandUp)
{
  initial_state(falling)
  {
    transition
    {
      if (theFallDownState.state == FallDownState::flying)
      {
        goto standing;
      }
      else if (theFallDownState.standUpOnlyWhenLyingStill)
      {
        if (theFallDownState.state == FallDownState::onGroundLyingStill)
          goto lying;
      }
      else
      {
        if (theFallDownState.state == FallDownState::onGround)
          goto lying;
      }
    }
    action {}
  }

  state(lying)
  {
    transition
    {
      if (theFallDownState.direction == FallDownState::back)
        goto lyingOnBack;
      else if (theFallDownState.direction == FallDownState::front)
        goto lyingOnFront;
      else if (theMotionRequest.GoalieIsDiving && (theFallDownState.direction == FallDownState::left || theFallDownState.direction == FallDownState::right))
        goto lyingOnSideGoalie;
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
      theMotionRequest.GoalieIsDiving = false;
      SpecialAction(theMotionState.standUpStatus.bestStandUpMotionBack, false);
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
      theMotionRequest.GoalieIsDiving = false;
      SpecialAction(theMotionState.standUpStatus.bestStandUpMotionFront, false);
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

  state(lyingOnSideGoalie)
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
      SpecialAction(SpecialActionRequest::standUpSideNaoGoalie, theFallDownState.direction == FallDownState::right);
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
      else if (theSpecialActionsOutput.isFallProtectionNeeded)
        goto falling;
      else
        goto standing;
    }
    action {}
  }

  target_state(standing)
  {
    transition
    {
      if (theFallDownState.state != FallDownState::flying)
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
        else if (theSpecialActionsOutput.isFallProtectionNeeded)
          goto falling;
      }
    }
    action {}
  }
}
