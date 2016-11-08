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
      else if (theFallDownState.direction == FallDownState::left
        || theFallDownState.direction == FallDownState::right)
        goto lyingOnSide;
    }
    action
    {
    }
  }

  state(lyingOnBack)
  {
    transition
    {
      if (theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::standUpBackNao)
      {
        goto standing;
      }
    }
    action
    {
      SpecialAction(SpecialActionRequest::standUpBackNao, false);
    }
  }

  state(lyingOnFront)
  {
    transition
    {
      if (theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::standUpFrontNao)
      {
        goto standing;
      }
    }
    action
    {
	    SpecialAction(SpecialActionRequest::standUpFrontNao,false);
    }
  }

  state(lyingOnSide)
  {
    transition
    {
      if (state_time > 1000)
      {
        if (theFallDownState.direction == FallDownState::back)
          goto lyingOnBack;
        else if (theFallDownState.direction == FallDownState::front)
          goto lyingOnFront;
      }
      else if (state_time > 3000)
      {
        goto standing;
      }
    }
    action
    {
      SpecialAction(SpecialActionRequest::standUpSideNao,theFallDownState.direction == FallDownState::right);
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
        else if (theFallDownState.direction == FallDownState::left
          || theFallDownState.direction == FallDownState::right)
          goto lyingOnSide;
      }
    }
    action
    {
      Walk(WalkRequest::speed, 0,0,0);
    }
  }
}