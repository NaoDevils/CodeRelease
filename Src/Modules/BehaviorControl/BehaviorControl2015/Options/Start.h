option(Start)
{
  initial_state(playDead)
  {
    transition
    {
      if (SystemCall::getMode() == SystemCall::simulatedRobot)
        goto startBehavior; // Don't wait for the button in SimRobot

      // Skip playDead state at a restart after a crash
      else if (Global::getSettings().recover)
        goto startBehavior;

      // libbhuman starts transition to bhuman (chest button was pressed once)
      if (theRobotInfo.transitionToBhuman > 0.f)
        goto startBehavior;
    }
    
    action
    {
      SpecialAction(SpecialActionRequest::playDead);
    }
  }

  state(startBehavior)
  {
    transition
    {
      // libbhuman stopped bhuman (chest button was pressed three times)
      if (theRobotInfo.transitionToBhuman == 0.f)
        goto sitDown;
    }
    action
    {
      SoundControl();
      LEDControl();
      BodyControl();
      SwitchBehaviorState();
      theHeadControlRequest.controlType = HeadControlRequest::soccer;
    }
  }

  state(sitDown)
  {
    transition
    {
      if (state_time > 4000)
        goto stopBehavior;
    }
    action
    {
      SpecialAction(SpecialActionRequest::sitDown);
    }
  }

  state(stopBehavior)
  {
    transition
    {
      // libbhuman starts transition to bhuman (chest button was pressed once)
      if (theRobotInfo.transitionToBhuman > 0.f)
        goto startBehavior;
    }
    action
    {
      SpecialAction(SpecialActionRequest::playDead);
    }
  }
}