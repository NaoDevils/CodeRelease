option(SwitchBehaviorState)
{
  initial_state(notPressed)
  {
    transition
    {
      if (theKeyStates.pressed[KeyStates::chest]) // chest button pressed
        goto pressedOnce;
    }
    action
    {
     
    }
  }


  state(pressedOnce)
  {
    transition
    {
      if (state_time < 500 && !theKeyStates.pressed[KeyStates::chest])
        goto releasedOnce;
      else if (state_time >= 500)
        goto notPressed;
    }
    action
    {
      
    }
  }

  state(releasedOnce)
  {
    transition
    {
      if (state_time < 500 && theKeyStates.pressed[KeyStates::chest])
        goto pressedTwice;
      else if (state_time >= 500)
        goto notPressed;
    }
    action
    {

    }
  }

  state(pressedTwice)
  {
    transition
    {
      if (state_time < 500 && !theKeyStates.pressed[KeyStates::chest])
        goto releasedTwice;
      else if (state_time >= 500)
        goto notPressed;
    }
      action
    {

    }
  }

  state(releasedTwice)
  {
    transition
    {
      if (state_time < 500 && theKeyStates.pressed[KeyStates::chest])
        goto switchBehaviorState;
      else if (state_time >= 500)
        goto notPressed;
    }
    action
    {

    }
  }

  target_state(switchBehaviorState)
  {
    transition
    {
      if (state_time > 500)
        goto notPressed;
    }
    action
    {
      
    }
  }

}