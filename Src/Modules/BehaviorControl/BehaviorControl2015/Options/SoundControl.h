// TODO BH2015 port - no soundrequest available

option(SoundControl)
{
  initial_state(none)
  {
    transition
    {
      if (firstReady && theGameInfo.state == STATE_READY)
        goto ready;
      if (theRobotInfo.penalty != PENALTY_NONE)
        goto penalized;
      else
        goto none;
    }
    
    action
    {
      //localSoundRequest.sound = SoundRequest::none;
    }
  }

  state(ready)
  {
    transition
    {
      if (state_time > 28000)
      {
        goto none;
      }
    }
    action
    {
      if (theBehaviorData.role == BehaviorData::keeper)
        PlaySound("vader_imperial_march.wav");
    }
  }

  state(penalized)
  {
    transition
    {
      if (state_time > 2000)
        goto none;
    }
    action
    {
      //localSoundRequest.sound = SoundRequest::penalized;
    }
  }
}