option(Start)
{
  initial_state(playDead)
  {
    transition
    {
      if (SystemCall::getMode() == SystemCall::simulatedRobot)
        goto standUp; // Don't wait for the button in SimRobot

      if (action_done) // chest button pressed and released
        goto standUp;

      // Skip playDead state at a restart after a crash
      else if (Global::getSettings().recover)
        goto standUp;
    }
    
    action
    {
      SpecialAction(SpecialActionRequest::playDead);
      ButtonPressedAndReleased(KeyStates::chest, 1000, 0);
    }
  }
  

  state(standUp)
  {
    transition
    {
      if(action_done || state_time > 5000)
        goto startBehavior;
    }
    
    action
    {
      SpecialAction(SpecialActionRequest::stand);
    }
  }

  state(startBehavior)
  {
    transition
    {
      if (action_done)
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
      if (action_done)
      {
        goto standUp;
      }
    }
    action
    {
      SpecialAction(SpecialActionRequest::playDead);
      SwitchBehaviorState();
    }
  }
}