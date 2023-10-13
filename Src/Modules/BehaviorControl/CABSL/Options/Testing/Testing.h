// Testing Joints

option(Testing)
{
  initial_state(startTesting)
  {
    transition
    {
      if (state_time > 1000)
        goto jointTesting;
    }
    action
    {
      theBehaviorData.behaviorState = BehaviorData::testingJoints;
    }
  }

  state(jointTesting)
  {
    transition
    {
      if (state_time > 300000)
        goto endTesting;
    }
    action
    {
      SpecialAction(SpecialActionRequest::test, false);
    }
  }

  target_state(endTesting)
  {
    transition {}
    action
    {
      theBehaviorData.behaviorState = BehaviorData::game;
    }
  }
}
