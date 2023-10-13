/*
** Option to control stafety behavior differently for different malfunctions.
*/
option(RobotSafetyControl)
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
      if (!theMotionState.jointStatus.usableLegs)
        goto safety_shutdown;
    }
    action {}
  }

  state(safety_shutdown)
  {
    transition
    {
      if (action_done)
        goto safety_shutdown;
    }
    action
    {
      RobotSafetyShutdown();
    }
  }
}
