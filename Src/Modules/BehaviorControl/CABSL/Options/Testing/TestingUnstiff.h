// Testing Joints part 2

option(TestingUnstiff)
{
  initial_state(startTestingUnstiff)
  {
    transition
    {
      if (state_time > 1000)
        goto jointTestingUnstiff;
    }
    action
    {
      theBehaviorData.behaviorState = BehaviorData::testingJoints;
    }
  }

  state(jointTestingUnstiff)
  {
    transition
    {
      if (state_time > 300000)
        goto endTestingUnstiff;
    }
    action
    {
      SpecialAction(SpecialActionRequest::testUnstiff, false);
    }
  }

  target_state(endTestingUnstiff)
  {
    transition {}
    action
    {
      theBehaviorData.behaviorState = BehaviorData::game;
    }
  }
}
