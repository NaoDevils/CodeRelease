/*
** Option to control standUp differently for different roles.
** TODO: if wide stance motions are executed, do not accidently trigger standup directly!
*/
option(StandUpControl)
{
  common_transition
  {
    if ((theGameInfo.state != STATE_PLAYING && theGameInfo.state != STATE_SET && theGameInfo.state != STATE_READY) || theRobotInfo.penalty != PENALTY_NONE)
      goto wait;
  }

  initial_state(wait)
  {
    transition
    {
      if ((theGameInfo.state == STATE_PLAYING || theGameInfo.state == STATE_SET || theGameInfo.state == STATE_READY) && theRobotInfo.penalty == PENALTY_NONE)
        goto standing;
    }
    action {}
  }

  state(standing)
  {
    transition
    {
      if (theFallDownState.state != FallDownState::upright && theFallDownState.state != FallDownState::falling && theFallDownState.state != FallDownState::standingUp
          && !theMotionInfo.inStandUpMotion() && !theMotionInfo.inBlockMotion())
        goto standUp;
    }
    action {}
  }

  state(standUp)
  {
    transition
    {
      if (action_done)
        goto standing;
    }
    action
    {
      StandUp();
      theBehaviorData.soccerState = BehaviorData::standUp;
    }
  }
}
