
option(BodyControlPS)
{
  theBehaviorData.soccerState = BehaviorData::positioning;
  common_transition
  {
    if (theGameInfo.state == STATE_INITIAL)
      goto initial;
    else if (theGameInfo.state == STATE_FINISHED)
      goto finished;
    else
    {
      if (theRoleSymbols.role != BehaviorData::keeper
          && ((theFallDownState.state != FallDownState::upright && theFallDownState.state != FallDownState::falling) || theMotionInfo.inBlockMotion() || theMotionInfo.inStandUpMotion()))
      {
        goto stand_up;
      }
      else
      {
        if (theGameInfo.state == STATE_SET)
          goto set;
        else if (theGameInfo.state == STATE_PLAYING)
          goto playing;
      }
    }
  }

  initial_state(initial)
  {
    action
    {
      if (theFallDownState.state == FallDownState::upright)
        SpecialAction(SpecialActionRequest::sitDown);
      else
        SpecialAction(SpecialActionRequest::playDead);
    }
  }

  state(set)
  {
    action
    {
      Walk(WalkRequest::speed, 0, 0, 0);
    }
  }

  state(playing)
  {
    action
    {

      if (theRoleSymbols.role == BehaviorData::keeper)
      {
        PenaltyKeeper();
      }
      else
      {
        PenaltyStriker();
      }
    }
  }

  state(finished)
  {
    action
    {
      SpecialAction(SpecialActionRequest::stand);
    }
  }

  state(stand_up)
  {
    action
    {
      StandUp();
    }
  }
}
