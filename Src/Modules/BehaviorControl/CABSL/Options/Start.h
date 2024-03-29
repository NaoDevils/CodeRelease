option(Start)
{
  common_transition
  {
    // check for malfunctions
    if (theBehaviorData.soccerState == BehaviorData::safetyShutdown)
    {
      goto malfunction;
    }
  }

  initial_state(playDead)
  {
    transition
    {
      if ((SystemCall::getMode() == SystemCall::simulatedRobot) // Don't wait for the button in SimRobot
          || (Global::getSettings().recover) // Skip playDead state at a restart after a crash
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
      // Penalty shootout and robot is penalized
      if ((Global::getSettings().gameMode == Settings::GameMode::penaltyShootout) && (theRobotInfo.penalty != PENALTY_NONE))
        goto sitDownPenalty;

      // ndevilsbase stopped framework (chest button was pressed three times or all head buttons pressed 1 sec)
      if (theRobotInfo.transitionToFramework == 0.f)
        goto sitDown;
    }
    action
    {
      if (Global::getSettings().gameMode == Settings::GameMode::penaltyShootout)
      {
        HeadControlPS();
        BodyControlPS();
        theBehaviorData.behaviorState = BehaviorData::BehaviorState::penaltyShootout;
      }
      else
      {
        HeadControl(); // Does only: theHeadControlRequest.controlType = HeadControlRequest::soccer;
        SoundControl(); // Does nothing yet but tells the role
        BodyControl(); // Handles all other parts of the game
        StandUpControl(); // Triggers stand up motions when robot falls
        RobotSafetyControl(); // Triggers safety motion when robot has a broken leg joint
        theBehaviorData.behaviorState = BehaviorData::BehaviorState::game;
      }
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

  state(sitDownPenalty)
  {
    transition
    {
      if (state_time > 4000)
        goto waitForNewStatePenalty;
    }
    action
    {
      if (theRobotInfo.penalty == PENALTY_MANUAL)
        SpecialAction(SpecialActionRequest::stand);
      else
        SpecialAction(SpecialActionRequest::sitDown);
      theBehaviorData.soccerState = BehaviorData::penalized;
      theBehaviorData.behaviorState = BehaviorData::BehaviorState::penaltyShootout;
    }
  }

  state(waitForNewStatePenalty)
  {
    transition
    {
      if (state_time > 500 && theRobotInfo.penalty == PENALTY_NONE)
        goto startBehavior;
    }
    action
    {
      if (theRobotInfo.penalty == PENALTY_MANUAL)
        SpecialAction(SpecialActionRequest::stand);
      else
        SpecialAction(SpecialActionRequest::sitDown);
      theBehaviorData.soccerState = BehaviorData::SoccerState::waiting;
      theBehaviorData.behaviorState = BehaviorData::BehaviorState::penaltyShootout;
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
