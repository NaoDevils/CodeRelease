option(BodyControl)
{
  common_transition
  {
    if (theRobotInfo.penalty != PENALTY_NONE)
      goto penalized;
    else if (theGameInfo.state == STATE_INITIAL)
      goto initial;
    else if (theGameInfo.state == STATE_FINISHED)
      goto finished;
    else
    {
      if (theGameInfo.state != STATE_SET &&
        ((theFallDownState.state != FallDownState::upright && theFallDownState.state != FallDownState::falling)
          || (theMotionInfo.motion == MotionRequest::specialAction
            && (theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::standUpBackNao
              || theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::standUpFrontNao
              || theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::standUpSideNao))))
      {
        goto stand_up;
      }
      else if (theGameInfo.state == STATE_READY)
      {
        goto ready;
      }
      else if (theGameInfo.state == STATE_SET)
      {
        goto set;
      }
      else if (theGameInfo.state == STATE_PLAYING)
      {
        goto playing;
      }
    }
  }
  
  initial_state(initial)
  {
    transition
    {

    }
    
    action
    {
      SpecialAction(SpecialActionRequest::stand);
    }
  }

  state(penalized)
  {
    transition
    {

    }
    
    action
    {
      SpecialAction(SpecialActionRequest::standHigh);
    }
  }

  state(ready)
  {
    transition
    {

    }
    
    action
    {
      Walk(WalkRequest::speed, 0,0,0);
    }
  }

  state(set)
  {
    transition
    {

    }
    
    action
    {
      Walk(WalkRequest::speed, 0,0,0);
    }
  }

  state(playing)
  {
    transition
    {

    }

    action
    {
      Striker();
    }
  }

  state(finished)
  {
    transition
    {

    }
    
    action
    {
      SpecialAction(SpecialActionRequest::stand);
    }
  }

  state(stand_up)
  {
    transition
    {

    }

    action
    {
      StandUp();
    }
  }
}
