option(Start)
{
  common_transition
  {
    // check for malfunctions
    if (theBehaviorData.soccerState == BehaviorData::safetyShutdown)
    {
      if (theRobotInfo.transitionToFramework == 0.f)
        goto sitDown;
      else
        goto malfunction;
    }
  }

  initial_state(playDead)
  {
    transition
    {
      if ((SystemCall::getMode() == SystemCall::simulatedRobot) // Don't wait for the button in SimRobot
          || (theRobotInfo.transitionToFramework > 0.f)) // ndevilsbase starts transition to framework (chest button was pressed once)
        goto startBehavior;
    }
    action
    {
      SpecialAction(SpecialActionRequest::sitDown);
      theBehaviorData.behaviorState = BehaviorData::game;
    }
  }

  state(startBehavior)
  {
    transition
    {
      // check for auto calibration trigger (head front + chest)
      if (theFrameInfo.getTimeSince(theKeySymbols.lastTimeNotPressed[KeyStates::headFront]) > 100
          && theFrameInfo.getTimeSince(theKeySymbols.lastTimeNotPressed[KeyStates::chest]) > 100 && theGameInfo.state == STATE_INITIAL)
      {
        SystemCall::text2Speech("Starting calibration");
        goto autoCalibrate;
      }
      if (theFrameInfo.getTimeSince(theKeySymbols.lastTimeNotPressed[KeyStates::headRear]) > 100
          && theFrameInfo.getTimeSince(theKeySymbols.lastTimeNotPressed[KeyStates::chest]) > 100 && theGameInfo.state == STATE_INITIAL)
      {
        SystemCall::text2Speech("Testing Joints");
        goto testing;
      }
      if (theFrameInfo.getTimeSince(theKeySymbols.lastTimeNotPressed[KeyStates::headMiddle]) > 100
          && theFrameInfo.getTimeSince(theKeySymbols.lastTimeNotPressed[KeyStates::chest]) > 100 && theGameInfo.state == STATE_INITIAL)
      {
        SystemCall::text2Speech("Testing Joints unstiff");
        goto testingUnstiff;
      }

      // ndevilsbase stopped framework (chest button was pressed three times or all head buttons pressed 1 sec)
      if (theRobotInfo.transitionToFramework == 0.f)
        goto sitDown;
    }
    action
    {
      HeadControl(); // Does only: theHeadControlRequest.controlType = HeadControlRequest::soccer;
      SoundControl(); // Does nothing yet but tells the role
      BodyControl(); // Handles all other parts of the game
      StandUpControl(); // Triggers stand up motions when robot falls
      RobotSafetyControl(); // Triggers safety motion when robot has a broken leg joint
      theBehaviorData.behaviorState = BehaviorData::BehaviorState::game;
    }
  }

  state(autoCalibrate)
  {
    transition
    {
      // ndevilsbase stopped framework (chest button was pressed three times or all head buttons pressed 1 sec)
      if (theRobotInfo.transitionToFramework == 0.f)
        goto sitDown;
      else if (action_done && state_time > 200) // calibration finished
        goto startBehavior;
    }
    action
    {
      RobotSafetyControl();
      if (theBehaviorData.soccerState != BehaviorData::safetyShutdown)
        AutoCalibrate();
    }
  }
  state(malfunction)
  {
    transition {}
    action
    {
      if (theFallDownState.state == FallDownState::upright)
      {
        SpecialAction(SpecialActionRequest::sitDown, false);
      }
      else
      {
        SpecialAction(SpecialActionRequest::playDead, false);
      }
    }
  }

  state(testing)
  {
    transition
    {
      // ndevilsbase stopped framework (chest button was pressed three times or all head buttons pressed 1 sec)
      if (theRobotInfo.transitionToFramework == 0.f)
        goto sitDown;
      else if (action_done && state_time > 200) // calibration finished
        goto startBehavior;
    }
    action
    {
      Testing();
    }
  }
  state(testingUnstiff)
  {
    transition
    {
      // ndevilsbase stopped framework (chest button was pressed three times or all head buttons pressed 1 sec)
      if (theRobotInfo.transitionToFramework == 0.f)
        goto sitDown;
      else if (action_done && state_time > 200) // calibration finished
        goto startBehavior;
    }
    action
    {
      TestingUnstiff();
    }
  }


  state(sitDown)
  {
    transition
    {
      if (state_time > 1000)
        goto stopBehavior;
    }
    action
    {
      SpecialAction(SpecialActionRequest::sitDown);
      theBehaviorData.soccerState = BehaviorData::waiting;
      theBehaviorData.behaviorState = BehaviorData::game;
    }
  }

  state(stopBehavior)
  {
    transition
    {
      // ndevilsbase starts transition to framework (chest button was pressed once)
      if (theRobotInfo.transitionToFramework > 0.f)
        goto startBehavior;
    }
    action
    {
      SpecialAction(SpecialActionRequest::sitDown);
      if (theBehaviorData.soccerState != BehaviorData::safetyShutdown)
        theBehaviorData.soccerState = BehaviorData::SoccerState::waiting;
      theBehaviorData.behaviorState = BehaviorData::game;
    }
  }
}
