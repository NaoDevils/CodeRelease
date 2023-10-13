/** This option lets the robot sit down in case of leg joint malfunktions. */
option(RobotSafetyShutdown)
{
  initial_state(broken_leg)
  {
    transition
    {
      if ((theBehaviorData.soccerState == BehaviorData::safetyShutdown || !theMotionState.jointStatus.usableLegs) && theBehaviorData.behaviorState != BehaviorData::testingJoints
          && theGameInfo.state != STATE_INITIAL)
      {
        goto error_message;
      }
    }
    action {}
  }


  target_state(error_message)
  {
    transition {}
    action
    {
      if (theBehaviorData.soccerState != BehaviorData::safetyShutdown)
      {
        SystemCall::playSound("alarm.wav");
        SystemCall::text2Speech("Haelp, Leg broken.");
        theBehaviorData.soccerState = BehaviorData::safetyShutdown;
      }
    }
  }
}
